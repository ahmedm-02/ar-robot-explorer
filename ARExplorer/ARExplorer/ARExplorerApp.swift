// ARExplorerApp.swift
// AR Explorer — Search & Rescue Research Project
//
// App entry point. Uses a standard SwiftUI lifecycle.
// ContentView owns the ARSessionManager as a @StateObject so the
// AR session lives for the full lifetime of the window.

import SwiftUI

@main
struct ARExplorerApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
    }
}
