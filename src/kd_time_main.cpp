#include "polyhedralGravity/input/YAMLConfigReader.h"
#include <benchmark/benchmark.h>

namespace polyhedralGravity {

    const std::vector<std::string> filePaths{"../example-config/data/Eros", "../example-config/data/Eros_scaled_1", "../example-config/data/Eros_scaled_2", "../example-config/data/Eros_scaled_3", "../example-config/data/Eros_scaled_4"};

    Polyhedron createBigPolyhedron(const PolyhedralSource &source, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        return Polyhedron(source, 1.0, NormalOrientation::OUTWARDS, PolyhedronIntegrity::HEAL, algorithm);
    }

    void BM_Polyhedron_Tree(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        TetgenAdapter adapter{{filePaths.at(state.range(0)) + ".node", filePaths.at(state.range(0)) + ".face"}};
        for (auto _: state) {
            Polyhedron polyhedron = createBigPolyhedron(adapter.getPolyhedralSource(), algorithm);
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(std::get<1>(adapter.getPolyhedralSource()).size()));
    }


    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronNoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE)->Arg(0);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronQuadratic", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->Arg(0);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->Range(0, 1);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronLog", PlaneSelectionAlgorithm::Algorithm::LOG)->Range(0, 4);
}// namespace polyhedralGravity

BENCHMARK_MAIN();