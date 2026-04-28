// ARViewContainer.swift
// AR Explorer — Search & Rescue Research Project
//
// UIViewRepresentable that owns the ARView lifecycle.
//
// Responsibilities:
//   • Create and configure the ARView + ARWorldTrackingConfiguration
//   • Attach ARCoachingOverlayView for onboarding guidance
//   • Register the Coordinator as ARSessionDelegate
//   • Forward tap gestures → raycast → sessionManager.placeMarker
//
// The Coordinator is created once and never replaced, which keeps the
// ARSession and gesture recognizer stable across SwiftUI re-renders.

import ARKit
import RealityKit
import SwiftUI

struct ARViewContainer: UIViewRepresentable {

    /// The shared state object. Plain `var` — @Observable handles tracking
    /// automatically through the parent ContentView's @State.
    var sessionManager: ARSessionManager

    // -----------------------------------------------------------------------
    // MARK: UIViewRepresentable
    // -----------------------------------------------------------------------

    func makeUIView(context: Context) -> ARView {
        // ── 1. Create the ARView ───────────────────────────────────────────
        // .nonAR would show a blank background; default (.ar) shows the camera.
        let arView = ARView(frame: .zero)

        // Give the session manager a (weak) handle so it can add/remove anchors
        // and clear the scene without coupling ARViewContainer to every action.
        sessionManager.arView = arView

        // ── 2. World-tracking configuration ───────────────────────────────
        let config = ARWorldTrackingConfiguration()

        // Detect both horizontal (floor, tables) and vertical (walls) planes.
        // This gives raycasts the best chance of hitting a surface.
        config.planeDetection = [.horizontal, .vertical]

        // Let RealityKit capture environment lighting for more realistic shading.
        // Falls back gracefully on devices that don't support it.
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            // LiDAR not available on iPhone 16 base model; skip depth semantics.
        }
        config.environmentTexturing = .automatic

        arView.session.run(config, options: [])

        // ── Start WebSocket server ─────────────────────────────────────────
        // Delayed 2 s so ARKit can initialise world tracking before the iOS
        // "Allow local network access?" permission prompt appears. Without the
        // delay the prompt interrupts ARKit mid-initialisation and the coaching
        // overlay gets stuck.
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.0) {
            sessionManager.startServer()
            sessionManager.startMJPEGServer()
        }

        // ── 3. Wire up the delegate ────────────────────────────────────────
        // The Coordinator implements ARSessionDelegate for tracking-state updates.
        arView.session.delegate = context.coordinator

        // ── 4. Coaching overlay ────────────────────────────────────────────
        // ARCoachingOverlayView guides the user to move the phone until ARKit
        // has enough feature points to initialise world tracking.
        setupCoachingOverlay(for: arView)

        // ── 5. Tap gesture (places a colored marker) ───────────────────────
        let tap = UITapGestureRecognizer(
            target: context.coordinator,
            action: #selector(Coordinator.handleTap(_:))
        )
        arView.addGestureRecognizer(tap)

        // ── 6. Long-press gesture (places the selected USDZ model) ─────────
        // 0.5 s hold time distinguishes it clearly from a normal tap.
        let longPress = UILongPressGestureRecognizer(
            target: context.coordinator,
            action: #selector(Coordinator.handleLongPress(_:))
        )
        longPress.minimumPressDuration = 0.5
        arView.addGestureRecognizer(longPress)

        // Optional debug: uncomment to visualise detected planes while developing.
        // arView.debugOptions = [.showAnchorGeometry, .showFeaturePoints]

        return arView
    }

    func updateUIView(_ uiView: ARView, context: Context) {
        // SwiftUI calls this whenever observed state changes.
        // The ARView is self-managing; no reconfiguration needed here.
        // The coordinator already holds a reference to sessionManager, so
        // published changes propagate automatically through the shared object.
    }

    func makeCoordinator() -> Coordinator {
        Coordinator(sessionManager: sessionManager)
    }

    // -----------------------------------------------------------------------
    // MARK: Coaching overlay helper
    // -----------------------------------------------------------------------

    private func setupCoachingOverlay(for arView: ARView) {
        let overlay = ARCoachingOverlayView()

        // Tie the overlay directly to the session so it auto-activates whenever
        // tracking quality drops (e.g. after app backgrounding).
        overlay.session = arView.session

        // `.tracking` dismisses the overlay as soon as world tracking is stable,
        // without waiting for a confirmed horizontal plane. Better for this app
        // because raycasting works on estimated planes even before a full plane
        // mesh is established.
        overlay.goal = .tracking
        overlay.activatesAutomatically = true

        // Pin the overlay to fill the ARView exactly.
        overlay.translatesAutoresizingMaskIntoConstraints = false
        arView.addSubview(overlay)
        NSLayoutConstraint.activate([
            overlay.topAnchor.constraint(equalTo: arView.topAnchor),
            overlay.bottomAnchor.constraint(equalTo: arView.bottomAnchor),
            overlay.leadingAnchor.constraint(equalTo: arView.leadingAnchor),
            overlay.trailingAnchor.constraint(equalTo: arView.trailingAnchor),
        ])
    }

    // -----------------------------------------------------------------------
    // MARK: Coordinator
    // -----------------------------------------------------------------------

    /// Acts as ARSessionDelegate and handles tap-to-place gestures.
    ///
    /// Created once by SwiftUI and reused for the lifetime of the view.
    class Coordinator: NSObject, ARSessionDelegate {

        var sessionManager: ARSessionManager

        init(sessionManager: ARSessionManager) {
            self.sessionManager = sessionManager
        }

        // ── Tap-to-Place ────────────────────────────────────────────────────

        @objc func handleTap(_ recognizer: UITapGestureRecognizer) {
            guard let arView = recognizer.view as? ARView else { return }

            let tapLocation = recognizer.location(in: arView)

            // Try three targets in order of accuracy, stopping at the first hit.
            //
            // 1. .existingPlaneGeometry  — hits the actual scanned mesh of a
            //    detected plane. Most accurate but only works within the scanned area.
            //
            // 2. .existingPlaneInfinite  — extends any detected plane to infinity,
            //    so tapping the floor far away (e.g. across a room) still lands on
            //    the same floor plane even if that far edge hasn't been scanned yet.
            //
            // 3. .estimatedPlane        — falls back to a fresh estimate from
            //    nearby feature points. Useful for surfaces not yet formalised as
            //    an ARPlaneAnchor (e.g. a new wall you just pointed at).
            //
            // Note: ceilings and soft surfaces (couches, rugs) are not detectable
            // by camera-only tracking — LiDAR would be required for those.
            let targets: [(ARRaycastQuery.Target, ARRaycastQuery.TargetAlignment)] = [
                (.existingPlaneGeometry, .any),
                (.existingPlaneInfinite, .any),
                (.estimatedPlane,        .any),
            ]

            for (target, alignment) in targets {
                let results = arView.raycast(from: tapLocation, allowing: target, alignment: alignment)
                if let hit = results.first {
                    sessionManager.placeMarker(at: hit.worldTransform, in: arView)
                    return
                }
            }
            // All three failed — surface not yet detected. The coaching overlay
            // will prompt the user to scan more of the environment.
        }

        // ── Long-Press-to-Place Model ───────────────────────────────────────

        @objc func handleLongPress(_ recognizer: UILongPressGestureRecognizer) {
            // Only act on the initial "began" state, not on subsequent moved/ended events.
            guard recognizer.state == .began,
                  let arView = recognizer.view as? ARView else { return }

            let location = recognizer.location(in: arView)

            // Same three-stage raycast cascade as handleTap.
            let targets: [(ARRaycastQuery.Target, ARRaycastQuery.TargetAlignment)] = [
                (.existingPlaneGeometry, .any),
                (.existingPlaneInfinite, .any),
                (.estimatedPlane,        .any),
            ]

            for (target, alignment) in targets {
                let results = arView.raycast(from: location, allowing: target, alignment: alignment)
                if let hit = results.first {
                    sessionManager.placeModel(at: hit.worldTransform, in: arView)
                    return
                }
            }
        }

        // ── ARSessionDelegate ────────────────────────────────────────────────
        // ARKit dispatches all delegate methods on the main thread (verified in
        // Apple's docs), so we can update @Published properties directly.

        func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
            sessionManager.updateTrackingStatus(for: camera)
        }

        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            sessionManager.pushFrame(frame)
        }

        func session(_ session: ARSession, didFailWithError error: Error) {
            sessionManager.trackingStatus = "Session Error: \(error.localizedDescription)"
        }

        func sessionWasInterrupted(_ session: ARSession) {
            sessionManager.trackingStatus = "Session Interrupted"
        }

        func sessionInterruptionEnded(_ session: ARSession) {
            // Let ARKit relocalize on its own — do NOT force .resetTracking here.
            // Forcing a reset would restart plane detection from zero, re-trigger
            // the coaching overlay, and lose all world-space anchors. ARKit's
            // built-in relocalization is fast and keeps existing anchors intact.
            sessionManager.trackingStatus = "Tracking: Resuming…"
        }
    }
}
