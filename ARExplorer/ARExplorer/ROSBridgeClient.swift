// ROSBridgeClient.swift
// AR Explorer — Search & Rescue Research Project
//
// WebSocket CLIENT that connects to a rosbridge_websocket server (ROS 2) and
// subscribes to a geometry_msgs/PointStamped topic.  When a message arrives the
// client calls onPosition(x:y:z:) on the main thread so that ARSessionManager
// can place a marker in the scene.
//
// Uses URLSessionWebSocketTask (Apple built-in) — no third-party dependencies.
// The iPhone is the CLIENT here; the existing NWListener server keeps running
// in parallel so the MacBook GUI continues to work simultaneously.

import Foundation
import Observation

// ---------------------------------------------------------------------------
// MARK: - ROSConnectionStatus
// ---------------------------------------------------------------------------

enum ROSConnectionStatus: Equatable {
    case disconnected
    case connecting
    case connected
    case error(String)

    var displayText: String {
        switch self {
        case .disconnected:          return "Disconnected"
        case .connecting:            return "Connecting…"
        case .connected:             return "Connected"
        case .error(let msg):        return "Error: \(msg)"
        }
    }

    var isConnected: Bool {
        if case .connected = self { return true }
        return false
    }
}

// ---------------------------------------------------------------------------
// MARK: - ROSBridgeClient
// ---------------------------------------------------------------------------

/// Connects to a rosbridge_websocket server and subscribes to
/// `/ar_marker_position` (geometry_msgs/PointStamped).
///
/// Lifecycle:
///   1. Call `connect(host:port:)` — creates the WebSocket, sends a subscribe
///      frame, and starts the receive loop.
///   2. Set `onPosition` to handle incoming point messages.
///   3. Call `disconnect()` to tear down cleanly.
///
/// All `connectionStatus` mutations and `onPosition` calls are dispatched to
/// the main queue so callers never need to marshal themselves.
@Observable
final class ROSBridgeClient {

    // MARK: - Observed state (drives SwiftUI)

    var connectionStatus: ROSConnectionStatus = .disconnected
    /// The "ws://host:port" string shown in the HUD.
    var serverURL: String = ""

    // MARK: - Callback

    /// Called on the **main thread** with (x, y, z) from each PointStamped
    /// message that arrives on the subscribed topic.
    var onPosition: ((Float, Float, Float) -> Void)?

    // MARK: - Constants

    static let defaultPort = 9090
    private let topic      = "/ar_marker_position"
    private let msgType    = "geometry_msgs/PointStamped"

    // MARK: - Private state
    // @ObservationIgnored keeps these out of @Observable tracking — required
    // for types that can't participate in observation synthesis.

    @ObservationIgnored
    private var webSocketTask: URLSessionWebSocketTask?
    @ObservationIgnored
    private var urlSession: URLSession?

    // MARK: - Connect / Disconnect

    /// Open a connection to the rosbridge server at `host:port`.
    /// Safe to call from the main thread (which SwiftUI button actions use).
    func connect(host: String, port: Int = ROSBridgeClient.defaultPort) {
        tearDown()                        // cancel any existing connection first

        let urlString = "ws://\(host):\(port)"
        serverURL     = urlString

        guard let url = URL(string: urlString) else {
            connectionStatus = .error("Invalid URL: \(urlString)")
            return
        }

        connectionStatus = .connecting

        let session  = URLSession(configuration: .default)
        urlSession   = session
        let task     = session.webSocketTask(with: url)
        webSocketTask = task
        task.resume()   // initiates TCP + WebSocket handshake asynchronously

        // Send the rosbridge subscribe message right away; URLSession will
        // queue the frame until the handshake completes.
        sendSubscribe(task: task)

        // Begin the receive loop — each call to receive() is one-shot so we
        // chain them in the completion handler.
        scheduleReceive(task: task)
    }

    /// Close the connection gracefully.
    func disconnect() {
        tearDown()
        connectionStatus = .disconnected
    }

    // MARK: - Private helpers

    private func tearDown() {
        webSocketTask?.cancel(with: .normalClosure, reason: nil)
        webSocketTask = nil
        urlSession    = nil
    }

    /// Send the rosbridge v2 "subscribe" operation for our topic.
    private func sendSubscribe(task: URLSessionWebSocketTask) {
        let payload: [String: Any] = [
            "op":    "subscribe",
            "topic": topic,
            "type":  msgType
        ]
        guard let data = try? JSONSerialization.data(withJSONObject: payload),
              let text = String(data: data, encoding: .utf8) else {
            print("[ROSBridgeClient] Failed to encode subscribe message")
            return
        }

        task.send(.string(text)) { [weak self] error in
            DispatchQueue.main.async {
                guard let self else { return }
                if let error {
                    print("[ROSBridgeClient] Subscribe send error: \(error.localizedDescription)")
                    self.connectionStatus = .error(error.localizedDescription)
                } else if case .connecting = self.connectionStatus {
                    // First frame went through → TCP + WS handshake is complete.
                    self.connectionStatus = .connected
                }
            }
        }
    }

    /// Schedule one asynchronous receive.  Chains itself on success so the loop
    /// runs until the task is cancelled or an error occurs.
    private func scheduleReceive(task: URLSessionWebSocketTask) {
        task.receive { [weak self] result in
            guard let self else { return }

            switch result {
            case .success(let message):
                self.handleMessage(message)
                // URLSessionWebSocketTask is one-shot per receive call —
                // we must re-arm it for the next frame.
                self.scheduleReceive(task: task)

            case .failure(let error):
                let nsErr = error as NSError
                // NSURLErrorCancelled / code 57 ("socket not connected") are
                // normal when we deliberately cancel the task — don't surface them.
                let isNormalClose = (nsErr.domain == NSURLErrorDomain &&
                                     nsErr.code   == NSURLErrorCancelled) ||
                                    nsErr.code == 57
                if !isNormalClose {
                    print("[ROSBridgeClient] Receive error: \(error.localizedDescription)")
                    DispatchQueue.main.async {
                        self.connectionStatus = .error(error.localizedDescription)
                    }
                }
            }
        }
    }

    /// Decode a WebSocket frame and, if it is a rosbridge publish on our
    /// topic, extract the PointStamped coordinates and call `onPosition`.
    private func handleMessage(_ message: URLSessionWebSocketTask.Message) {
        // Normalise to a String — rosbridge always sends UTF-8 JSON text frames,
        // but handle binary frames defensively.
        let text: String
        switch message {
        case .string(let s):
            text = s
        case .data(let d):
            guard let s = String(data: d, encoding: .utf8) else {
                print("[ROSBridgeClient] Received non-UTF-8 binary frame — ignoring")
                return
            }
            text = s
        @unknown default:
            return
        }

        // Parse the outer JSON envelope
        guard
            let data = text.data(using: .utf8),
            let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any]
        else {
            print("[ROSBridgeClient] JSON parse failure: \(text.prefix(200))")
            return
        }

        // Only act on publish messages for our specific topic
        guard
            let op         = json["op"]    as? String, op    == "publish",
            let msgTopic   = json["topic"] as? String, msgTopic == topic,
            let msg        = json["msg"]   as? [String: Any],
            let point      = msg["point"]  as? [String: Any]
        else { return }

        // JSONSerialization returns NSNumber for all JSON numeric types
        // (integer or floating-point), so we use NSNumber → floatValue to
        // handle both "x": 1 and "x": 1.0 without extra guards.
        let x = (point["x"] as? NSNumber)?.floatValue ?? 0
        let y = (point["y"] as? NSNumber)?.floatValue ?? 0
        let z = (point["z"] as? NSNumber)?.floatValue ?? 0

        DispatchQueue.main.async {
            self.onPosition?(x, y, z)
        }
    }
}
