// ARSessionManager.swift
// AR Explorer — Search & Rescue Research Project
//it w
// Observable object that owns all mutable AR state:
//   • tracking status string  (drives the status label)
//   • current marker type     (drives the toggle button)
//   • placed anchor registry  (needed for "Clear All")
//
// The class is NOT marked @MainActor because ARKit already calls every
// session-delegate method on the main thread. updateTrackingStatus(for:)
// is therefore safe to call directly from the Coordinator delegate.

import ARKit
import CoreImage
import Observation
import RealityKit
import SwiftUI
import UIKit

// ---------------------------------------------------------------------------
// MARK: - MarkerType
// ---------------------------------------------------------------------------

/// The two kinds of markers the user can place.
///
/// • `.redBox`    → "searched area" marker  (red cube,   10 cm)
/// • `.blueSphere` → "hazard" marker         (blue sphere, 5 cm radius)
///
/// Extend this enum to add more marker types in the future.
enum MarkerType: String, CaseIterable {
    case redBox     = "Red Box"
    case blueSphere = "Blue Sphere"

    /// Label for the toggle button — describes what the *next* tap will place.
    var nextLabel: String {
        switch self {
        case .redBox:      return "Placing: Red Box"
        case .blueSphere:  return "Placing: Blue Sphere"
        }
    }

    /// SF Symbol name used in the toggle button.
    var symbolName: String {
        switch self {
        case .redBox:      return "cube.fill"
        case .blueSphere:  return "circle.fill"
        }
    }

    /// Color used both for the 3-D mesh material and the button tint.
    var uiColor: UIColor {
        switch self {
        case .redBox:      return .systemRed
        case .blueSphere:  return .systemBlue
        }
    }
}

// ---------------------------------------------------------------------------
// MARK: - ARSessionManager
// ---------------------------------------------------------------------------

@Observable
class ARSessionManager {

    // MARK: Observed state (consumed by SwiftUI views)
    // With @Observable, properties are tracked automatically — no @Published needed.

    /// Human-readable tracking status shown in the HUD label.
    var trackingStatus: String = "Tracking: Initializing…"

    /// Which marker type the next tap will place.
    var currentMarkerType: MarkerType = .redBox

    /// How many objects have been placed (useful for future UI/reporting).
    var placedCount: Int = 0

    // MARK: Network state (consumed by ContentView's network status panel)

    /// Human-readable WebSocket connection status.
    var connectionStatus: String = "No client connected"

    /// True while a MacBook client has an open WebSocket connection.
    var isClientConnected: Bool = false

    /// "192.168.x.x:8080" — shown in the HUD so you know what to type in the terminal.
    var serverAddress: String = "Starting…"

    /// "192.168.x.x:8082/stream" — MJPEG endpoint the ASUS pulls for AprilTag detection.
    var mjpegStreamAddress: String = "Starting…"

    // MARK: Internal state
    // @ObservationIgnored excludes these from the @Observable macro's tracking —
    // required for `weak var` (which can't be synthesised as a computed property)
    // and for private backing arrays / objects SwiftUI doesn't need to watch.

    /// Weak reference to the live ARView; set by ARViewContainer.makeUIView.
    @ObservationIgnored
    weak var arView: ARView?

    /// Every AnchorEntity we have added to the scene, kept so "Clear All" can
    /// remove them without iterating the entire scene graph.
    @ObservationIgnored
    private var placedAnchors: [AnchorEntity] = []

    /// Maps ROS marker IDs to their anchors, enabling per-ID deletion.
    @ObservationIgnored
    private var rosMarkerIDs: [(key: String, anchor: AnchorEntity)] = []

    /// The WebSocket server. Started once from ARViewContainer.makeUIView.
    @ObservationIgnored
    private let server = WebSocketServer()

    /// USDZ model catalogue and entity cache — shared with ContentView's picker strip.
    /// Marked @ObservationIgnored because the reference never changes after init;
    /// SwiftUI observes ModelManager's own @Observable properties directly.
    @ObservationIgnored
    let modelManager = ModelManager()

    /// ROS 2 bridge client — connects to a rosbridge_websocket server as a
    /// WebSocket client (parallel to the existing NWListener server).
    /// @ObservationIgnored for the same reason as modelManager: the reference
    /// never changes; ContentView observes ROSBridgeClient's own properties.
    @ObservationIgnored
    let rosBridge = ROSBridgeClient()

    /// MJPEG HTTP server on port 8082 — serves the ARKit camera feed to the ASUS
    /// for AprilTag detection in shared-frame calibration.
    @ObservationIgnored
    private let mjpegServer = MJPEGServer()

    /// Reused CIContext for JPEG encoding. Thread-safe; expensive to create.
    @ObservationIgnored
    private let ciContext = CIContext(options: [.useSoftwareRenderer: false])

    /// Serial background queue for JPEG encoding so we never block the main thread
    /// (ARKit delegate callbacks land there). Serial keeps frame order intact and
    /// prevents pile-up if encoding falls behind capture.
    @ObservationIgnored
    private let encodeQueue = DispatchQueue(label: "MJPEGEncoder", qos: .userInitiated)

    /// Monotonic timestamp of the last frame we kicked off encoding for. Used to
    /// throttle the publish rate to ~`mjpegTargetFPS` regardless of ARKit's fps.
    @ObservationIgnored
    private var lastEncodeTime: CFTimeInterval = 0

    /// True while a frame is encoding. We skip new frames while it is set so a
    /// slow encode doesn't queue an unbounded backlog of CVPixelBuffers.
    @ObservationIgnored
    private var encodingInFlight: Bool = false

    /// Target output rate matches the RealSense MJPEG stream (12 fps).
    private let mjpegTargetFPS: Double = 12

    // MARK: - Public API

    /// Cycle to the next marker type.
    func toggleMarkerType() {
        let all = MarkerType.allCases
        let idx = all.firstIndex(of: currentMarkerType) ?? 0
        currentMarkerType = all[(idx + 1) % all.count]
    }

    /// Remove every placed object from the scene and reset the counter.
    func clearAllObjects() {
        guard let arView else { return }
        for anchor in placedAnchors {
            arView.scene.removeAnchor(anchor)
        }
        placedAnchors.removeAll()
        rosMarkerIDs.removeAll()
        placedCount = 0
    }

    /// Create a marker at `worldTransform` and add it to the scene.
    ///
    /// Called by the Coordinator after a successful raycast hit.
    func placeMarker(at worldTransform: simd_float4x4, in arView: ARView) {
        let anchor = AnchorEntity(world: worldTransform)
        anchor.addChild(makeEntity(for: currentMarkerType))
        arView.scene.addAnchor(anchor)
        placedAnchors.append(anchor)
        placedCount += 1
    }

    /// Place the currently selected USDZ model at `worldTransform`.
    ///
    /// Called by the Coordinator after a successful long-press raycast hit.
    /// Falls back to an orange sphere if no model is selected or loading fails.
    func placeModel(at worldTransform: simd_float4x4, in arView: ARView) {
        let name = modelManager.selectedModelName ?? ""
        placeModelEntity(named: name, scale: 1.0, label: name, at: worldTransform, in: arView)
    }

    // MARK: - ROS Bridge client

    /// Connect to a rosbridge_websocket server and wire up callbacks
    /// for both PointStamped and Marker topics.  The existing NWListener server
    /// keeps running — both input channels work simultaneously.
    func connectToROS(host: String, port: Int = ROSBridgeClient.defaultPort) {
        // Legacy: /ar_marker_position (PointStamped) → green sphere
        rosBridge.onPosition = { [weak self] x, y, z in
            self?.placeRemoteMarker(x: x, y: y, z: z,
                                    label: "ROS",
                                    color: "green",
                                    radius: 0.08)
        }

        // New: /ar_markers (visualization_msgs/Marker) → full marker support
        rosBridge.onMarker = { [weak self] marker in
            self?.handleROSMarker(marker)
        }

        rosBridge.connect(host: host, port: port)
    }

    // MARK: - ROS Marker handling

    /// Map a ROSMarker.Action into scene operations.
    private func handleROSMarker(_ marker: ROSMarker) {
        switch marker.action {
        case .add:
            // Remove existing marker with this ID to prevent stacking
            deleteROSMarker(id: marker.id)

            let anchorCountBefore = placedAnchors.count
            if marker.shapeType == .meshResource, !marker.meshResource.isEmpty {
                placeRemoteModel(
                    x: marker.x, y: marker.y, z: marker.z,
                    modelName: marker.meshResource,
                    label: marker.label,
                    scale: marker.scale
                )
            } else {
                let colorName = closestColorName(r: marker.r, g: marker.g, b: marker.b)
                placeRemoteMarker(
                    x: marker.x, y: marker.y, z: marker.z,
                    label: marker.label,
                    color: colorName,
                    radius: 0.07 * marker.scale
                )
            }
            // Track the newly placed anchor by ROS marker ID for deletion
            if placedAnchors.count > anchorCountBefore,
               let newAnchor = placedAnchors.last {
                rosMarkerIDs.append((key: "ros_marker_\(marker.id)", anchor: newAnchor))
            }

        case .delete:
            deleteROSMarker(id: marker.id)

        case .deleteAll:
            clearAllObjects()
        }
    }

    /// Delete a specific ROS-placed marker by its ID.
    private func deleteROSMarker(id: Int) {
        guard let arView else { return }
        let key = "ros_marker_\(id)"
        if let index = rosMarkerIDs.firstIndex(where: { $0.key == key }) {
            let anchor = rosMarkerIDs[index].anchor
            arView.scene.removeAnchor(anchor)
            placedAnchors.removeAll { $0 === anchor }
            rosMarkerIDs.remove(at: index)
            placedCount = max(0, placedCount - 1)
        }
    }

    /// Best-effort mapping from RGB floats to a named color string.
    private func closestColorName(r: Float, g: Float, b: Float) -> String {
        // Simple heuristic: pick the named color with the smallest distance.
        let candidates: [(String, Float, Float, Float)] = [
            ("red",    1, 0, 0),    ("green",  0, 1, 0),
            ("blue",   0.2, 0.4, 1),("yellow", 1, 1, 0),
            ("orange", 1, 0.5, 0),  ("white",  1, 1, 1),
            ("cyan",   0, 1, 1),    ("purple", 0.6, 0.2, 1),
            ("pink",   1, 0.4, 0.7),
        ]
        var best = "green"
        var bestDist: Float = .greatestFiniteMagnitude
        for (name, cr, cg, cb) in candidates {
            let d = (r - cr) * (r - cr) + (g - cg) * (g - cg) + (b - cb) * (b - cb)
            if d < bestDist { bestDist = d; best = name }
        }
        return best
    }

    /// Disconnect from the rosbridge server.
    func disconnectFromROS() {
        rosBridge.disconnect()
    }

    // MARK: - WebSocket server

    /// Start the WebSocket server and wire up its callbacks.
    /// Called once from ARViewContainer.makeUIView after the AR session starts.
    func startServer() {
        serverAddress = "\(WebSocketServer.localIPAddress()):\(server.port)"

        server.onConnectionChange = { [weak self] connected in
            self?.isClientConnected = connected
            self?.connectionStatus  = connected ? "MacBook connected" : "No client connected"
            if connected { self?.sendModelList() }
        }

        server.onCommand = { [weak self] command in
            switch command {
            case .place(let x, let y, let z, let label, let color):
                self?.placeRemoteMarker(x: x, y: y, z: z, label: label, color: color)
            case .placeModel(let x, let y, let z, let modelName, let label, let scale):
                self?.placeRemoteModel(x: x, y: y, z: z, modelName: modelName, label: label, scale: scale)
            case .clear:
                self?.clearAllObjects()
            }
        }

        server.start()
    }

    // MARK: - MJPEG camera server

    /// Start the MJPEG HTTP server. Called from ARViewContainer.makeUIView.
    /// Independent of the WebSocket server so the two can fail/restart separately.
    func startMJPEGServer() {
        mjpegStreamAddress = "\(WebSocketServer.localIPAddress()):\(mjpegServer.port)/stream"
        mjpegServer.start()
    }

    /// Convert an ARKit frame to JPEG and broadcast it to MJPEG clients.
    /// Called from the ARSessionDelegate on the main thread for every captured frame.
    /// Throttles to `mjpegTargetFPS`, encodes off-thread, and skips frames if a
    /// previous encode is still in progress.
    func pushFrame(_ frame: ARFrame) {
        guard mjpegServer.hasClients else { return }

        let now = CACurrentMediaTime()
        let minInterval = 1.0 / mjpegTargetFPS
        if now - lastEncodeTime < minInterval { return }
        if encodingInFlight { return }
        lastEncodeTime = now
        encodingInFlight = true

        // CVPixelBuffer is reference-counted in Swift, so capturing it in the closure
        // retains it across the dispatch hop. ARKit's frame buffer pool is bounded;
        // releasing eagerly (by exiting the closure) keeps the pool flowing.
        let pixelBuffer = frame.capturedImage

        encodeQueue.async { [weak self] in
            guard let self else { return }
            let jpeg = self.encodeJPEG(pixelBuffer: pixelBuffer)
            DispatchQueue.main.async {
                self.encodingInFlight = false
                if let jpeg {
                    self.mjpegServer.pushFrame(jpeg)
                }
            }
        }
    }

    /// CVPixelBuffer (ARKit YUV, landscape sensor orientation) → 640x480 JPEG.
    /// Output resolution and orientation match what `iphone_apriltag_processor.py`
    /// assumes (intrinsics fx=fy=500, cx=320, cy=240 are calibrated for 640x480).
    private func encodeJPEG(pixelBuffer: CVPixelBuffer) -> Data? {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let extent = ciImage.extent
        guard extent.width > 0, extent.height > 0 else { return nil }

        let targetSize = CGSize(width: 640, height: 480)
        let scaleX = targetSize.width / extent.width
        let scaleY = targetSize.height / extent.height
        let scale = min(scaleX, scaleY)
        let scaled = ciImage.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
        let cropped = scaled.cropped(to: CGRect(origin: .zero, size: targetSize))

        let qualityKey = CIImageRepresentationOption(
            rawValue: kCGImageDestinationLossyCompressionQuality as String
        )
        return ciContext.jpegRepresentation(
            of: cropped,
            colorSpace: CGColorSpaceCreateDeviceRGB(),
            options: [qualityKey: 0.7]
        )
    }

    // MARK: - Remote marker placement

    /// Transform a camera-relative offset into a world-space position and place
    /// a labeled marker there.
    ///
    /// Coordinate convention (matches standard ARKit camera space):
    ///   +x = right of camera
    ///   +y = above camera
    ///   -z = in front of camera   (so z = -3.0 → 3 m ahead)
    func placeRemoteMarker(x: Float, y: Float, z: Float, label: String, color: String, radius: Float = 0.07) {
        guard let arView,
              let frame = arView.session.currentFrame else {
            print("[ARSessionManager] No AR frame — cannot place remote marker")
            return
        }

        // camera.transform is the 4×4 matrix that maps camera space → world space.
        // Multiplying the homogeneous camera-space point (x, y, z, 1) by this
        // matrix gives us the corresponding world-space position.
        let camTransform = frame.camera.transform
        let camPoint     = SIMD4<Float>(x, y, z, 1.0)
        let worldPos4    = camTransform * camPoint
        let worldPos     = SIMD3<Float>(worldPos4.x, worldPos4.y, worldPos4.z)

        // Build anchor + sphere entity
        let anchor = AnchorEntity(world: worldPos)

        var mat = SimpleMaterial()
        mat.color     = SimpleMaterial.BaseColor(tint: resolvedColor(color), texture: nil)
        mat.roughness = 0.4
        mat.metallic  = 0.1

        // Remote markers are larger than tap-placed ones for visual distinction.
        // Default: 7 cm (MacBook GUI). ROS markers pass 8 cm explicitly.
        let sphere = ModelEntity(mesh: .generateSphere(radius: radius), materials: [mat])
        anchor.addChild(sphere)

        // Optional floating text label above the sphere
        if !label.isEmpty, let labelEntity = makeLabelEntity(label) {
            anchor.addChild(labelEntity)
        }

        arView.scene.addAnchor(anchor)
        placedAnchors.append(anchor)
        placedCount += 1
    }

    // MARK: - Remote model placement

    /// Transform a camera-relative offset into world space and place a USDZ model there.
    func placeRemoteModel(x: Float, y: Float, z: Float, modelName: String, label: String, scale: Float) {
        guard let arView,
              let frame = arView.session.currentFrame else {
            print("[ARSessionManager] No AR frame — cannot place remote model")
            return
        }
        let camTransform = frame.camera.transform
        let worldPos4    = camTransform * SIMD4<Float>(x, y, z, 1.0)
        let worldPos     = SIMD3<Float>(worldPos4.x, worldPos4.y, worldPos4.z)
        var transform    = matrix_identity_float4x4
        transform.columns.3 = SIMD4<Float>(worldPos.x, worldPos.y, worldPos.z, 1.0)
        placeModelEntity(named: modelName, scale: scale, label: label, at: transform, in: arView)
    }

    // MARK: - ARSessionDelegate helpers

    /// Translate ARKit's camera tracking state into a user-friendly string.
    ///
    /// Called directly from Coordinator.session(_:cameraDidChangeTrackingState:)
    /// which ARKit dispatches on the main thread.
    func updateTrackingStatus(for camera: ARCamera) {
        switch camera.trackingState {
        case .normal:
            trackingStatus = "Tracking: Normal"

        case .notAvailable:
            trackingStatus = "Tracking: Not Available"

        case .limited(let reason):
            switch reason {
            case .initializing:
                trackingStatus = "Tracking: Limited — Initializing"
            case .relocalizing:
                trackingStatus = "Tracking: Limited — Relocalizing"
            case .excessiveMotion:
                trackingStatus = "Tracking: Limited — Excessive Motion"
            case .insufficientFeatures:
                trackingStatus = "Tracking: Limited — Insufficient Features"
            @unknown default:
                trackingStatus = "Tracking: Limited"
            }

        @unknown default:
            trackingStatus = "Tracking: Unknown"
        }
    }

    // MARK: - Private helpers

    /// Send the list of bundled USDZ model names to the connected client.
    /// Called automatically when a new WebSocket client connects.
    private func sendModelList() {
        let payload: [String: Any] = [
            "action": "model_list",
            "models": modelManager.availableModels
        ]
        guard let data = try? JSONSerialization.data(withJSONObject: payload),
              let text = String(data: data, encoding: .utf8) else { return }
        server.send(text)
    }

    /// Create a floating text label entity that always faces the camera
    /// (BillboardComponent) positioned above the sphere.
    private func makeLabelEntity(_ text: String) -> ModelEntity? {
        // Font size is in metres; 4 cm is clearly legible at arm's length.
        let font = CTFontCreateWithName("Helvetica-Bold" as CFString, 0.04, nil)
        guard let mesh = try? MeshResource.generateText(
            text,
            extrusionDepth: 0.002,   // almost-flat — looks clean when billboarding
            font: font,
            containerFrame: .zero,
            alignment: .center,
            lineBreakMode: .byWordWrapping
        ) else { return nil }

        var mat = SimpleMaterial()
        mat.color     = SimpleMaterial.BaseColor(tint: .white, texture: nil)
        mat.roughness = 1.0
        mat.metallic  = 0.0

        let entity = ModelEntity(mesh: mesh, materials: [mat])

        // Centre the text horizontally over the sphere, 12 cm above its centre.
        let textWidth = mesh.bounds.extents.x
        entity.position = [-textWidth / 2, 0.12, 0]

        // BillboardComponent makes this entity always rotate to face the camera.
        entity.components.set(BillboardComponent())
        return entity
    }

    /// Map a color name string to a UIColor.
    private func resolvedColor(_ name: String) -> UIColor {
        switch name.lowercased() {
        case "red":    return .systemRed
        case "green":  return .systemGreen
        case "blue":   return .systemBlue
        case "yellow": return .systemYellow
        case "orange": return .systemOrange
        case "purple": return .systemPurple
        case "white":  return .white
        case "cyan":   return .cyan
        case "pink":   return .systemPink
        default:       return .systemOrange
        }
    }

    /// Load (or clone from cache) a USDZ entity, apply scale and label, and add it to the scene.
    ///
    /// If `name` is empty or the file isn't in the bundle, falls back to a small orange sphere
    /// so the placement point is still visible and the counter increments correctly.
    private func placeModelEntity(named name: String,
                                  scale: Float,
                                  label: String,
                                  at worldTransform: simd_float4x4,
                                  in arView: ARView) {
        let anchor = AnchorEntity(world: worldTransform)

        if !name.isEmpty, let entity = try? modelManager.loadEntity(named: name) {
            entity.scale = SIMD3<Float>(repeating: scale)
            anchor.addChild(entity)
        } else {
            // Fallback: orange sphere — visible even when no USDZ files are bundled.
            var mat = SimpleMaterial()
            mat.color = SimpleMaterial.BaseColor(tint: .systemOrange, texture: nil)
            anchor.addChild(ModelEntity(mesh: .generateSphere(radius: 0.05), materials: [mat]))
            if !name.isEmpty {
                print("[ARSessionManager] Model '\(name).usdz' not found — showing fallback sphere")
            }
        }

        if !label.isEmpty, let labelEntity = makeLabelEntity(label) {
            anchor.addChild(labelEntity)
        }

        arView.scene.addAnchor(anchor)
        placedAnchors.append(anchor)
        placedCount += 1
    }

    /// Build the RealityKit entity for the given tap-placed marker type.
    private func makeEntity(for type: MarkerType) -> ModelEntity {
        // Use a physically-based but unlit-looking material so the marker is
        // clearly visible regardless of scene lighting.
        var material = SimpleMaterial()
        material.color = SimpleMaterial.BaseColor(tint: type.uiColor, texture: nil)
        material.roughness = 0.8
        material.metallic  = 0.0

        let mesh: MeshResource
        switch type {
        case .redBox:
            // 10 cm cube — clearly visible at arm's length
            mesh = MeshResource.generateBox(size: 0.10)
        case .blueSphere:
            // 5 cm radius sphere
            mesh = MeshResource.generateSphere(radius: 0.05)
        }

        return ModelEntity(mesh: mesh, materials: [material])
    }
}
