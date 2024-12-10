#include "polyhedralGravity/input/YAMLConfigReader.h"
#include <benchmark/benchmark.h>

namespace polyhedralGravity {

    PolyhedralSource polyhedralSource{};

    Polyhedron createBigPolyhedron(const PlaneSelectionAlgorithm::Algorithm &algorithm) {
        return Polyhedron(polyhedralSource, 1.0, NormalOrientation::OUTWARDS, PolyhedronIntegrity::HEAL, algorithm);
    }

    void BM_Polyhedron_Tree(benchmark::State &state, const polyhedralGravity::PlaneSelectionAlgorithm::Algorithm &algorithm) {
        for (auto _: state) {
            createBigPolyhedron(algorithm);
        }
    }

    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronNoTree", PlaneSelectionAlgorithm::Algorithm::NOTREE);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronQuadratic", PlaneSelectionAlgorithm::Algorithm::QUADRATIC);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronLogSquared", PlaneSelectionAlgorithm::Algorithm::LOGSQUARED);
    BENCHMARK_CAPTURE(BM_Polyhedron_Tree, "BigPolyhedronLog", PlaneSelectionAlgorithm::Algorithm::LOG);
}// namespace polyhedralGravity

int main(int argc, char **argv) {
    using namespace polyhedralGravity;
    if (argc < 2) {
        std::cout << "Error: Please supply a config yaml argument!" << std::endl;
        return EXIT_FAILURE;
    }
    const std::shared_ptr<ConfigSource> config = std::make_shared<YAMLConfigReader>(argv[1]);
    polyhedralSource = config->getDataSource()->getPolyhedralSource();
    ::benchmark::Initialize(&argc, argv);
    ::benchmark::RunSpecifiedBenchmarks();
}