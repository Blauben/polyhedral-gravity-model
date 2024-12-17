#include "polyhedralGravity/input/YAMLConfigReader.h"
#include <benchmark/benchmark.h>

namespace polyhedralGravity {

    Polyhedron createBigPolyhedron(const PolyhedralSource &source, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        return Polyhedron(source, 1.0, NormalOrientation::OUTWARDS, PolyhedronIntegrity::HEAL, algorithm);
    }

    void BM_Polyhedron_Tree(benchmark::State &state, const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        const std::vector<std::string> configPaths{"../example-config/example_eros.yaml", "../example-config/example_eros_scaled_1.yaml", "../example-config/example_eros_scaled_2.yaml", "../example-config/example_eros_scaled_3.yaml", "../example-config/example_eros_scaled_4.yaml"};
        const std::shared_ptr<ConfigSource> config = std::make_shared<YAMLConfigReader>(configPaths.at(state.range(0)));
        const PolyhedralSource source = config->getDataSource()->getPolyhedralSource();
        for (auto _: state) {
            Polyhedron polyhedron = createBigPolyhedron(source, algorithm);
            benchmark::ClobberMemory();
        }
        state.SetComplexityN(static_cast<benchmark::ComplexityN>(std::get<1>(source).size()));
    }


    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronNoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE)->Arg(0);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronQuadratic", PlaneSelectionAlgorithm::Algorithm::QUADRATIC)->Arg(0);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED)->Range(0, 1);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronLog", PlaneSelectionAlgorithm::Algorithm::LOG)->Range(0, 4);
}// namespace polyhedralGravity

BENCHMARK_MAIN();