#include "polyhedralGravity/input/YAMLConfigReader.h"
#include <benchmark/benchmark.h>

namespace polyhedralGravity {
    const std::vector<std::string> filePaths{
        "../example-config/data/Eros_downscaled-2", "../example-config/data/Eros_downscaled-1",
        "../example-config/data/Eros", "../example-config/data/Eros_upscaled-1",
        "../example-config/data/Eros_upscaled-2"
    };

    Polyhedron createBigPolyhedron(const PolyhedralFiles &files,
                                   const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        return {files, 1.0, NormalOrientation::OUTWARDS, PolyhedronIntegrity::DISABLE, algorithm};
    }

    void BM_Polyhedron_Tree(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        const PolyhedralFiles polyhedralFiles{{filePaths.at(state.range(0)) + ".node", filePaths.at(state.range(0)) + ".face"}};
        Polyhedron polyhedron = createBigPolyhedron(polyhedralFiles, algorithm);
        for (auto _: state) {
            auto result = polyhedron.checkPlaneUnitNormalOrientation();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(polyhedron.countFaces()));
    }

    void BM_Polyhedron_Tree_Twice(benchmark::State &state) {
        const PolyhedralFiles polyhedralFiles = {filePaths.at(state.range(0)) + ".node", filePaths.at(state.range(0)) + ".face"};
        Polyhedron initializedPolyhedron = createBigPolyhedron(polyhedralFiles, PlaneSelectionAlgorithm::Algorithm::LOG);
        initializedPolyhedron.prebuildKDTree();
        for (auto _: state) {
            auto result = initializedPolyhedron.checkPlaneUnitNormalOrientation();
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(initializedPolyhedron.countFaces()));
    }

    void BM_Polyhedron_Tree_Build(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        const PolyhedralFiles polyhedralFiles = {filePaths.at(state.range(0)) + ".node", filePaths.at(state.range(0)) + ".face"};
        const auto initializedPolyhedron = Polyhedron(polyhedralFiles, 1.0, NormalOrientation::OUTWARDS, PolyhedronIntegrity::DISABLE, algorithm);
        auto workload = [initializedPolyhedron](){initializedPolyhedron.prebuildKDTree(); return "finished";};
        for (auto _: state) {
            benchmark::DoNotOptimize(workload());
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(initializedPolyhedron.countFaces()));
    }

    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronNoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE)->DenseRange(
        0, static_cast<long>(filePaths.size() - 1), 1);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronQuadratic",
                      PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->DenseRange(0, static_cast<long>(filePaths.size() - 1), 1);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronLogSquared",
                      PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->DenseRange(0, static_cast<long>(filePaths.size() - 1), 1);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronLog", PlaneSelectionAlgorithm::Algorithm::LOG)->DenseRange(
        0, static_cast<long>(filePaths.size() - 1), 1);
    BENCHMARK(BM_Polyhedron_Tree_Twice)->Name("BigPolyhedronSecondRun")->DenseRange(
        0, static_cast<long>(filePaths.size() - 1), 1);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree_Build, "BigPolyhedronBuildTreeSquared", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->DenseRange(
        0, static_cast<long>(filePaths.size() - 1), 1);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree_Build, "BigPolyhedronBuildTreeLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->DenseRange(
        0, static_cast<long>(filePaths.size() - 1), 1);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree_Build, "BigPolyhedronBuildTreeLog", PlaneSelectionAlgorithm::Algorithm::LOG)->DenseRange(
        0, static_cast<long>(filePaths.size() - 1), 1);
} // namespace polyhedralGravity

BENCHMARK_MAIN();
