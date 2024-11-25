#include "time_executable.h"

#include "polyhedralGravity/input/TetgenAdapter.h"

namespace polyhedralGravity {
    void measureTreePerformance(std::vector<Array3> vertices, std::vector<IndexArray3> faces) {
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
            SPDLOG_LOGGER_INFO(PolyhedralGravityLogger::DEFAULT_LOGGER.getLogger(), "Function {} execution started", funcStructPtr->name);
            const auto duration_ms = funcStructPtr->measureTimeMs();
            SPDLOG_LOGGER_INFO(PolyhedralGravityLogger::DEFAULT_LOGGER.getLogger(), "{} execution time: {} ms", funcStructPtr->name, duration_ms);
        });
    }
}// namespace polyhedralGravity