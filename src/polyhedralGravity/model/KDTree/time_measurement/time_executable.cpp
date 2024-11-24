#include "time_executable.h"

#include "polyhedralGravity/input/TetgenAdapter.h"

namespace polyhedralGravity {
    void measureTreePerformance() {
        TetgenAdapter adapter{std::vector<std::string>{"resources/GravityModelBigTest.node", "resources/GravityModelBigTest.face"}};
        auto [vertices, faces] = adapter.getPolyhedralSource();
        auto createBigPolyhedron = [&vertices, &faces](const PlaneSelectionAlgorithm::Algorithm &algorithm) {
            return Polyhedron{vertices, faces,
                              1.0,
                              NormalOrientation::OUTWARDS,
                              PolyhedronIntegrity::HEAL,
                              algorithm};
        };

        const std::vector<std::shared_ptr<IFunction>> functions{
                std::make_shared<Function<Polyhedron, PlaneSelectionAlgorithm::Algorithm>>("BigPolyhedronNoTree", createBigPolyhedron, PlaneSelectionAlgorithm::Algorithm::NOTREE),
                std::make_shared<Function<Polyhedron, PlaneSelectionAlgorithm::Algorithm>>("BigPolyhedronQuadratic", createBigPolyhedron, PlaneSelectionAlgorithm::Algorithm::QUADRATIC),
                std::make_shared<Function<Polyhedron, PlaneSelectionAlgorithm::Algorithm>>("BigPolyhedronLogSquared", createBigPolyhedron, PlaneSelectionAlgorithm::Algorithm::LOGSQUARED),
                std::make_shared<Function<Polyhedron, PlaneSelectionAlgorithm::Algorithm>>("BigPolyhedronLog", createBigPolyhedron, PlaneSelectionAlgorithm::Algorithm::LOG)};

        std::for_each(functions.begin(), functions.end(), [](const auto funcStructPtr) {
            const auto duration_ms = funcStructPtr->measureTimeMs();
            std::cout << funcStructPtr->name << " execution time: " << duration_ms << " ms" << std::endl;
        });
    }
}// namespace polyhedralGravity