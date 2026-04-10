// ModelManager.swift
// AR Explorer — Search & Rescue Research Project
//
// Responsibilities:
//   • Scan the app bundle at launch for all .usdz files
//   • Expose the list of model names for the picker UI
//   • Load and cache Entity objects so the same file isn't read from disk
//     more than once — subsequent placements clone the cached entity
//
// Usage:
//   let entity = try modelManager.loadEntity(named: "toy_car")
//   entity.scale = SIMD3<Float>(repeating: 0.5)
//   anchor.addChild(entity)

import Foundation
import Observation
import RealityKit

@Observable
final class ModelManager {

    // MARK: Observed state (drives the picker strip in ContentView)

    /// Names of all .usdz files found in the app bundle (without extension), sorted.
    var availableModels: [String] = []

    /// The model name currently selected in the picker — used by the next long-press.
    var selectedModelName: String? = nil

    // MARK: Private cache

    /// Loaded entities keyed by model name. Never observed — SwiftUI doesn't need to watch it.
    @ObservationIgnored
    private var entityCache: [String: Entity] = [:]

    // MARK: Init

    init() {
        scanBundle()
    }

    // MARK: - Bundle scan

    /// Finds every .usdz file at the root of the app bundle and populates `availableModels`.
    private func scanBundle() {
        let urls = Bundle.main.urls(forResourcesWithExtension: "usdz", subdirectory: nil) ?? []
        let names = urls
            .map { $0.deletingPathExtension().lastPathComponent }
            .sorted()
        availableModels = names
        selectedModelName = names.first   // auto-select the first model if any exist
    }

    // MARK: - Entity loading

    /// Load a USDZ entity by name (without the .usdz extension).
    ///
    /// The first call for a given name loads from disk and caches the result.
    /// Subsequent calls return a deep clone of the cached entity — each placed
    /// instance is independent so animations and transforms don't share state.
    ///
    /// Throws `ModelError.notFound` if the file doesn't exist in the bundle.
    func loadEntity(named name: String) throws -> Entity {
        // Return a clone of the cached entity if we've loaded this before.
        if let cached = entityCache[name] {
            return cached.clone(recursive: true)
        }

        guard let url = Bundle.main.url(forResource: name, withExtension: "usdz") else {
            throw ModelError.notFound(name)
        }

        // Entity.load(contentsOf:) is synchronous and fine for bundled assets.
        // For very large models, wrap this call in Task.detached if you need
        // the main thread to stay completely free during loading.
        let entity = try Entity.load(contentsOf: url)
        entityCache[name] = entity
        return entity.clone(recursive: true)
    }

    // MARK: - Errors

    enum ModelError: LocalizedError {
        case notFound(String)

        var errorDescription: String? {
            switch self {
            case .notFound(let name):
                return "'\(name).usdz' was not found in the app bundle. " +
                       "Drag the file into Xcode and make sure it's added to the ARExplorer target."
            }
        }
    }
}
