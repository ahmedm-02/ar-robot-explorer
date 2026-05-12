// ContentView.swift
// AR Explorer — Search & Rescue Research Project
//
// Root SwiftUI view. Composes:
//   • ARViewContainer  — full-screen ARView (camera + 3D scene)
//   • HUD overlay      — tracking label, Clear-All button, Toggle-Marker button
//
// The ARSessionManager is created here as a @StateObject so it lives for
// the full lifetime of the window (not recreated on re-renders).

import SwiftUI
import UIKit

struct ContentView: View {

    @State private var sessionManager = ARSessionManager()
    @AppStorage("rosHostname") private var rosHostname: String = ""

    var body: some View {
        ZStack(alignment: .top) {

            // ── AR View (fills the entire screen) ─────────────────────────
            ARViewContainer(sessionManager: sessionManager)
                .ignoresSafeArea()

            // ── HUD Overlay ───────────────────────────────────────────────
            VStack {
                trackingStatusLabel
                Spacer()
                networkStatusPanel
                rosBridgePanel
                modelPickerStrip
                controlBar
            }
        }
        // Keep the screen on while the app is in use — important for AR sessions.
        .onAppear {
            UIApplication.shared.isIdleTimerDisabled = true
            let host = rosHostname.trimmingCharacters(in: .whitespaces)
            if !host.isEmpty && !sessionManager.rosBridge.connectionStatus.isConnected {
                sessionManager.connectToROS(host: host)
            }
        }
        .onDisappear { UIApplication.shared.isIdleTimerDisabled = false }
    }

    // -----------------------------------------------------------------------
    // MARK: Sub-views
    // -----------------------------------------------------------------------

    /// Pill-shaped label at the top of the screen showing ARKit tracking quality.
    private var trackingStatusLabel: some View {
        Text(sessionManager.trackingStatus)
            .font(.caption.weight(.medium))
            .foregroundStyle(.white)
            .padding(.horizontal, 14)
            .padding(.vertical, 7)
            .background(.black.opacity(0.55), in: Capsule())
            // Animate text changes so sudden state flips feel smooth.
            .animation(.easeInOut(duration: 0.3), value: sessionManager.trackingStatus)
            .padding(.top, 16)
    }

    /// Panel for connecting to a rosbridge_websocket server (ROS 2).
    ///
    /// Sits directly below the MacBook network status panel so both input
    /// channels are visible at a glance.  Shows:
    ///   • colour-coded connection status dot + text
    ///   • the subscribed topic (read-only)
    ///   • hostname/IP text field + Connect button  (when disconnected)
    ///   • Disconnect button                        (when connected)
    private var rosBridgePanel: some View {
        let ros = sessionManager.rosBridge
        return VStack(alignment: .leading, spacing: 6) {

            // ── Header ──────────────────────────────────────────────────
            HStack(spacing: 5) {
                Image(systemName: "antenna.radiowaves.left.and.right")
                    .font(.caption.weight(.semibold))
                Text("ROS Bridge")
                    .font(.caption.weight(.semibold))
            }

            // ── Status indicator ─────────────────────────────────────────
            HStack(spacing: 6) {
                Circle()
                    .fill(rosStatusColor(ros.connectionStatus))
                    .frame(width: 8, height: 8)
                Text(ros.connectionStatus.displayText)
                    .font(.caption2)
            }

            // ── Subscribed topic (read-only) ─────────────────────────────
            Text("/ar_marker_position")
                .font(.caption2.monospaced())
                .foregroundStyle(.white.opacity(0.65))

            // ── Hostname input / action button ───────────────────────────
            HStack(spacing: 8) {
                if !ros.connectionStatus.isConnected {
                    TextField("IP address", text: $rosHostname)
                        .font(.caption2.monospaced())
                        .foregroundStyle(.white)
                        .padding(.horizontal, 8)
                        .padding(.vertical, 5)
                        .background(.white.opacity(0.15),
                                    in: RoundedRectangle(cornerRadius: 6))
                        .frame(maxWidth: 140)
                        .autocorrectionDisabled()
                        .textInputAutocapitalization(.never)
                        .keyboardType(.numbersAndPunctuation)

                    Button {
                        sessionManager.connectToROS(host: rosHostname.trimmingCharacters(in: .whitespaces))
                    } label: {
                        Text("Connect")
                            .font(.caption2.weight(.semibold))
                            .foregroundStyle(.white)
                            .padding(.horizontal, 10)
                            .padding(.vertical, 5)
                            .background(.green.opacity(0.85), in: Capsule())
                    }
                    .disabled(rosHostname.trimmingCharacters(in: .whitespaces).isEmpty)
                } else {
                    Button {
                        sessionManager.disconnectFromROS()
                    } label: {
                        Text("Disconnect")
                            .font(.caption2.weight(.semibold))
                            .foregroundStyle(.white)
                            .padding(.horizontal, 10)
                            .padding(.vertical, 5)
                            .background(.red.opacity(0.85), in: Capsule())
                    }
                }
            }
        }
        .foregroundStyle(.white)
        .padding(.horizontal, 14)
        .padding(.vertical, 10)
        .background(.black.opacity(0.55), in: RoundedRectangle(cornerRadius: 10))
        .padding(.bottom, 10)
        .animation(.easeInOut(duration: 0.25), value: ros.connectionStatus)
    }

    /// Map a ROSConnectionStatus to a HUD dot colour.
    private func rosStatusColor(_ status: ROSConnectionStatus) -> Color {
        switch status {
        case .disconnected: return .orange
        case .connecting:   return .yellow
        case .connected:    return .green
        case .error:        return .red
        }
    }

    /// Horizontal strip of model-name chips above the control bar.
    ///
    /// Visible only when at least one .usdz file is present in the bundle.
    /// Tap a chip to select that model — long-pressing the AR view will then
    /// place that model at the tapped surface location.
    @ViewBuilder
    private var modelPickerStrip: some View {
        let models = sessionManager.modelManager.availableModels
        if !models.isEmpty {
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 10) {
                    ForEach(models, id: \.self) { name in
                        let isSelected = sessionManager.modelManager.selectedModelName == name
                        Button {
                            sessionManager.modelManager.selectedModelName = name
                        } label: {
                            HStack(spacing: 4) {
                                if isSelected {
                                    Image(systemName: "checkmark")
                                        .font(.caption2.weight(.bold))
                                }
                                Text(name)
                                    .font(.caption.weight(.semibold))
                                    .lineLimit(1)
                            }
                            .foregroundStyle(.white)
                            .padding(.horizontal, 12)
                            .padding(.vertical, 8)
                            .background(
                                isSelected
                                    ? Color.blue.opacity(0.85)
                                    : Color.black.opacity(0.55),
                                in: Capsule()
                            )
                        }
                        .animation(.easeInOut(duration: 0.15), value: isSelected)
                    }
                }
                .padding(.horizontal, 14)
            }
            .padding(.bottom, 8)
        }
    }

    /// Network status panel showing the server address and connection state.
    private var networkStatusPanel: some View {
        VStack(alignment: .leading, spacing: 5) {
            // WiFi address — copy this into the terminal on your MacBook
            Label(sessionManager.serverAddress, systemImage: "wifi")
                .font(.caption2.monospaced().weight(.medium))

            // MJPEG endpoint — used by the ASUS for AprilTag detection
            Label(sessionManager.mjpegStreamAddress, systemImage: "video")
                .font(.caption2.monospaced().weight(.medium))

            // Connection dot + status text
            HStack(spacing: 6) {
                Circle()
                    .fill(sessionManager.isClientConnected ? Color.green : Color.orange)
                    .frame(width: 8, height: 8)
                Text(sessionManager.connectionStatus)
                    .font(.caption2)
            }
        }
        .foregroundStyle(.white)
        .padding(.horizontal, 14)
        .padding(.vertical, 10)
        .background(.black.opacity(0.55), in: RoundedRectangle(cornerRadius: 10))
        .padding(.bottom, 10)
        .animation(.easeInOut(duration: 0.25), value: sessionManager.isClientConnected)
    }

    /// Bottom control bar with two action buttons.
    private var controlBar: some View {
        HStack(spacing: 14) {

            // ── Clear All ─────────────────────────────────────────────────
            Button {
                sessionManager.clearAllObjects()
            } label: {
                Label("Clear All", systemImage: "trash")
                    .font(.subheadline.weight(.semibold))
                    .foregroundStyle(.white)
                    .padding(.horizontal, 18)
                    .padding(.vertical, 11)
                    .background(.red.opacity(0.85), in: Capsule())
            }

            // ── Toggle Marker Type ────────────────────────────────────────
            // Shows the *current* marker type so the user always knows what
            // they are about to place. Tap to switch to the other type.
            Button {
                sessionManager.toggleMarkerType()
            } label: {
                Label(
                    sessionManager.currentMarkerType.nextLabel,
                    systemImage: sessionManager.currentMarkerType.symbolName
                )
                .font(.subheadline.weight(.semibold))
                .foregroundStyle(.white)
                .padding(.horizontal, 18)
                .padding(.vertical, 11)
                .background(
                    Color(sessionManager.currentMarkerType.uiColor).opacity(0.85),
                    in: Capsule()
                )
            }
            // Animate the button color/label swap on toggle.
            .animation(.easeInOut(duration: 0.2), value: sessionManager.currentMarkerType)
        }
        .padding(.bottom, 36)  // Clear of the home-indicator bar
    }
}

// ---------------------------------------------------------------------------
// MARK: - Preview
// ---------------------------------------------------------------------------

#Preview {
    // ARKit does not run in the simulator or canvas preview;
    // this just confirms the UI compiles and lays out correctly.
    ContentView()
}
