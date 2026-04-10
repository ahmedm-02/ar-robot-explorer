// WebSocketServer.swift
// AR Explorer — Search & Rescue Research Project
//
// Lightweight WebSocket server built entirely on Apple's Network framework.
// No third-party iOS dependencies.
//
// Design decisions:
//   • NWListener + NWConnection both run on DispatchQueue.main so every
//     callback fires on the main thread. This avoids actor-isolation issues
//     with Xcode 26's SWIFT_DEFAULT_ACTOR_ISOLATION = MainActor setting and
//     is safe because our JSON payloads are tiny (never blocks the main thread).
//   • Only one client at a time. A new incoming connection cancels the old one.
//   • Callbacks (onCommand, onConnectionChange) are invoked on the main thread
//     so ARSessionManager can mutate @Observable state directly.

import Darwin       // getifaddrs, AF_INET, NI_MAXHOST
import Foundation
import Network

// ---------------------------------------------------------------------------
// MARK: - ARCommand
// ---------------------------------------------------------------------------

/// Decoded instruction sent by the MacBook client.
enum ARCommand: Sendable {
    /// Place a marker at a camera-relative offset with an optional label.
    case place(x: Float, y: Float, z: Float, label: String, color: String)
    /// Place a USDZ model at a camera-relative offset with an optional label and scale.
    case placeModel(x: Float, y: Float, z: Float, modelName: String, label: String, scale: Float)
    /// Remove all markers from the scene.
    case clear
}

// ---------------------------------------------------------------------------
// MARK: - WebSocketServer
// ---------------------------------------------------------------------------

final class WebSocketServer {

    // MARK: Configuration
    let port: UInt16

    // MARK: Callbacks — always called on the main thread
    var onCommand: ((ARCommand) -> Void)?
    var onConnectionChange: ((Bool) -> Void)?

    // MARK: Private
    private var listener: NWListener?
    private var activeConnection: NWConnection?

    init(port: UInt16 = 8080) {
        self.port = port
    }

    // -----------------------------------------------------------------------
    // MARK: Lifecycle
    // -----------------------------------------------------------------------

    func start() {
        // Build NWParameters: TCP with a WebSocket application protocol on top.
        // NWProtocolWebSocket handles the HTTP upgrade handshake automatically.
        let wsOptions = NWProtocolWebSocket.Options()
        wsOptions.autoReplyPing    = true       // framework handles ping/pong
        wsOptions.maximumMessageSize = 65_536   // 64 KB per frame — plenty for JSON

        let params = NWParameters.tcp
        params.defaultProtocolStack.applicationProtocols.insert(wsOptions, at: 0)

        do {
            listener = try NWListener(using: params, on: NWEndpoint.Port(rawValue: port)!)
        } catch {
            print("[WebSocketServer] Failed to create listener: \(error)")
            return
        }

        listener?.stateUpdateHandler = { state in
            switch state {
            case .ready:
                print("[WebSocketServer] Listening on port \(self.port)")
            case .failed(let error):
                print("[WebSocketServer] Listener failed: \(error)")
            default:
                break
            }
        }

        listener?.newConnectionHandler = { [weak self] connection in
            self?.accept(connection)
        }

        // Run everything on the main queue — keeps us on the main actor.
        listener?.start(queue: .main)
    }

    func stop() {
        listener?.cancel()
        activeConnection?.cancel()
        listener = nil
        activeConnection = nil
    }

    // -----------------------------------------------------------------------
    // MARK: Connection handling
    // -----------------------------------------------------------------------

    private func accept(_ connection: NWConnection) {
        // Cancel any existing client before accepting the new one.
        activeConnection?.cancel()
        activeConnection = connection

        connection.stateUpdateHandler = { [weak self] state in
            guard let self else { return }
            switch state {
            case .ready:
                self.onConnectionChange?(true)
                self.receive(on: connection)
            case .failed(let error):
                print("[WebSocketServer] Connection failed: \(error)")
                fallthrough
            case .cancelled:
                self.onConnectionChange?(false)
                if self.activeConnection === connection { self.activeConnection = nil }
            default:
                break
            }
        }

        connection.start(queue: .main)
    }

    // -----------------------------------------------------------------------
    // MARK: Receive loop
    // -----------------------------------------------------------------------

    /// Reads one WebSocket frame, processes it, then immediately re-arms
    /// for the next frame — forming an async callback chain on the main queue.
    private func receive(on connection: NWConnection) {
        connection.receiveMessage { [weak self] data, context, _, error in
            // Always re-arm unless the connection errored out.
            defer {
                if error == nil { self?.receive(on: connection) }
            }

            guard error == nil,
                  let data,
                  let context,
                  // Confirm this is a WebSocket text frame (not binary/ping/etc.)
                  let wsMeta = context.protocolMetadata(
                      definition: NWProtocolWebSocket.definition
                  ) as? NWProtocolWebSocket.Metadata,
                  wsMeta.opcode == .text,
                  let text = String(data: data, encoding: .utf8),
                  let command = WebSocketServer.parse(text)
            else { return }

            self?.onCommand?(command)
        }
    }

    // -----------------------------------------------------------------------
    // MARK: Send (for future use — e.g. ACK messages back to client)
    // -----------------------------------------------------------------------

    func send(_ text: String) {
        guard let connection = activeConnection,
              let data = text.data(using: .utf8) else { return }
        let meta = NWProtocolWebSocket.Metadata(opcode: .text)
        let ctx  = NWConnection.ContentContext(identifier: "ws-send", metadata: [meta])
        connection.send(content: data, contentContext: ctx, isComplete: true, completion: .idempotent)
    }

    // -----------------------------------------------------------------------
    // MARK: JSON parsing
    // -----------------------------------------------------------------------

    private static func parse(_ text: String) -> ARCommand? {
        guard
            let data   = text.data(using: .utf8),
            let json   = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
            let action = json["action"] as? String
        else { return nil }

        switch action {
        case "place":
            return .place(
                x:     (json["x"]     as? NSNumber)?.floatValue ?? 0,
                y:     (json["y"]     as? NSNumber)?.floatValue ?? 0,
                z:     (json["z"]     as? NSNumber)?.floatValue ?? 0,
                label: json["label"] as? String ?? "",
                color: json["color"] as? String ?? "green"
            )
        case "place_model":
            return .placeModel(
                x:         (json["x"]     as? NSNumber)?.floatValue ?? 0,
                y:         (json["y"]     as? NSNumber)?.floatValue ?? 0,
                z:         (json["z"]     as? NSNumber)?.floatValue ?? 0,
                modelName: json["model"]  as? String ?? "",
                label:     json["label"]  as? String ?? "",
                scale:     (json["scale"] as? NSNumber)?.floatValue ?? 1.0
            )
        case "clear":
            return .clear
        default:
            print("[WebSocketServer] Unknown action: '\(action)'")
            return nil
        }
    }

    // -----------------------------------------------------------------------
    // MARK: Local IP address
    // -----------------------------------------------------------------------

    /// Returns the device's current WiFi IPv4 address, or a fallback string.
    static func localIPAddress() -> String {
        // Check interfaces in priority order:
        //   en0       — WiFi (iPhone connected to a router)
        //   bridge100 — Personal Hotspot (iPhone IS the hotspot)
        //   en1..en9  — secondary WiFi / USB ethernet adapters
        let preferred = ["bridge100", "en0", "en1", "en2", "en3"]

        var ifaddr: UnsafeMutablePointer<ifaddrs>?
        guard getifaddrs(&ifaddr) == 0, let start = ifaddr else { return "Not available" }
        defer { freeifaddrs(start) }

        // Build a name→address map for all IPv4 interfaces.
        var found: [String: String] = [:]
        var ptr: UnsafeMutablePointer<ifaddrs>? = start
        while let iface = ptr {
            let name = String(cString: iface.pointee.ifa_name)
            if iface.pointee.ifa_addr.pointee.sa_family == UInt8(AF_INET),
               found[name] == nil {
                var host = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                getnameinfo(
                    iface.pointee.ifa_addr,
                    socklen_t(iface.pointee.ifa_addr.pointee.sa_len),
                    &host, socklen_t(host.count),
                    nil, 0, NI_NUMERICHOST
                )
                found[name] = String(cString: host)
            }
            ptr = iface.pointee.ifa_next
        }

        // Return the first match in priority order.
        for name in preferred {
            if let address = found[name] { return address }
        }
        return "Not on WiFi"
    }
}
