// MJPEGServer.swift
// AR Explorer — Search & Rescue Research Project
//
// HTTP MJPEG streaming server (multipart/x-mixed-replace) built on Network.framework.
// No third-party iOS dependencies — mirrors WebSocketServer.swift's NWListener-on-main
// pattern and runs in parallel with it.
//
// Why this exists:
//   The ASUS-side script `iphone_apriltag_processor.py` consumes the iPhone's camera
//   feed at http://<iphone>:8082/stream to detect AprilTags and publish the iPhone's
//   tag pose. Combined with the RealSense's tag pose, this yields the calibration
//   transform between the two frames (Phase 4 — AprilTag shared coordinate frame).
//
// Design decisions:
//   • Matches `scripts/stream_to_phone.py`'s wire format exactly (boundary=--frame,
//     part lines `--frame\r\n` then `Content-Type: image/jpeg\r\n` ...). Browsers,
//     FFmpeg, and OpenCV all consume that variant.
//   • Backpressure: if a client has an outstanding send when the next frame arrives,
//     we drop the new frame for that client rather than queueing — keeps memory bounded
//     on slow links.
//   • Encoding-cost guard: callers can check `hasClients` before doing JPEG work.

import Foundation
import Network

final class MJPEGServer {

    // MARK: Configuration
    let port: UInt16

    // MARK: Private
    private var listener: NWListener?
    private var clients: [ObjectIdentifier: Client] = [:]

    init(port: UInt16 = 8082) {
        self.port = port
    }

    // -----------------------------------------------------------------------
    // MARK: Lifecycle
    // -----------------------------------------------------------------------

    func start() {
        let params = NWParameters.tcp
        do {
            listener = try NWListener(using: params, on: NWEndpoint.Port(rawValue: port)!)
        } catch {
            print("[MJPEGServer] Failed to create listener: \(error)")
            return
        }

        listener?.stateUpdateHandler = { state in
            switch state {
            case .ready:
                print("[MJPEGServer] Listening on port \(self.port)")
            case .failed(let error):
                print("[MJPEGServer] Listener failed: \(error)")
            default:
                break
            }
        }

        listener?.newConnectionHandler = { [weak self] connection in
            self?.accept(connection)
        }

        listener?.start(queue: .main)
    }

    func stop() {
        listener?.cancel()
        for client in clients.values {
            client.connection.cancel()
        }
        clients.removeAll()
        listener = nil
    }

    // -----------------------------------------------------------------------
    // MARK: Streaming
    // -----------------------------------------------------------------------

    /// True if any client is connected. Use this to skip JPEG encoding when nobody
    /// is listening — the encode is the expensive part, not the send.
    var hasClients: Bool { !clients.isEmpty }

    /// Broadcast a JPEG-encoded frame to every connected client.
    /// Clients with an in-flight send skip this frame instead of queueing.
    func pushFrame(_ jpeg: Data) {
        guard !clients.isEmpty else { return }

        let header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: \(jpeg.count)\r\n\r\n"
        guard let headerData = header.data(using: .utf8) else { return }

        var packet = Data(capacity: headerData.count + jpeg.count + 2)
        packet.append(headerData)
        packet.append(jpeg)
        packet.append(contentsOf: [0x0D, 0x0A])  // trailing CRLF after the JPEG

        for (id, client) in clients {
            guard !client.sendInFlight else { continue }
            client.sendInFlight = true
            client.connection.send(content: packet, completion: .contentProcessed { [weak self] error in
                guard let self else { return }
                if let error {
                    print("[MJPEGServer] Send failed: \(error) — dropping client")
                    self.drop(id: id)
                    return
                }
                client.sendInFlight = false
            })
        }
    }

    // -----------------------------------------------------------------------
    // MARK: Connection handling
    // -----------------------------------------------------------------------

    private func accept(_ connection: NWConnection) {
        let client = Client(connection: connection)
        let id = ObjectIdentifier(client)

        connection.stateUpdateHandler = { [weak self] state in
            guard let self else { return }
            switch state {
            case .ready:
                self.handshake(client: client, id: id)
            case .failed(let error):
                print("[MJPEGServer] Connection failed: \(error)")
                self.drop(id: id)
            case .cancelled:
                self.drop(id: id)
            default:
                break
            }
        }

        connection.start(queue: .main)
    }

    /// Receive (and discard) the HTTP request bytes, then send the multipart preamble
    /// and register the client for streaming. We don't parse the request — any GET
    /// to any path returns the stream, matching `stream_to_phone.py`'s laxness.
    private func handshake(client: Client, id: ObjectIdentifier) {
        client.connection.receive(minimumIncompleteLength: 1, maximumLength: 4096) {
            [weak self] _, _, _, error in
            guard let self else { return }
            if error != nil {
                self.drop(id: id)
                return
            }

            let preamble =
                "HTTP/1.1 200 OK\r\n" +
                "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n" +
                "Cache-Control: no-cache, no-store, must-revalidate\r\n" +
                "Pragma: no-cache\r\n" +
                "Access-Control-Allow-Origin: *\r\n" +
                "Connection: close\r\n" +
                "\r\n"
            guard let data = preamble.data(using: .utf8) else { return }

            client.connection.send(content: data, completion: .contentProcessed { [weak self] error in
                guard let self else { return }
                if let error {
                    print("[MJPEGServer] Preamble send failed: \(error)")
                    self.drop(id: id)
                    return
                }
                self.clients[id] = client
                print("[MJPEGServer] Client connected (\(self.clients.count) total)")
            })
        }
    }

    private func drop(id: ObjectIdentifier) {
        if let client = clients.removeValue(forKey: id) {
            client.connection.cancel()
            print("[MJPEGServer] Client dropped (\(clients.count) remaining)")
        }
    }

    // -----------------------------------------------------------------------
    // MARK: Per-connection state
    // -----------------------------------------------------------------------

    final class Client {
        let connection: NWConnection
        var sendInFlight: Bool = false
        init(connection: NWConnection) { self.connection = connection }
    }
}
