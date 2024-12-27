#include "polyhedralGravity/model/KDTree/KDTree.h"

#include "polyhedralGravity/input/TetgenAdapter.h"
#include "polyhedralGravity/model/Polyhedron.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace polyhedralGravity {
    using testing::Contains;
    using testing::DoubleNear;
    using testing::ElementsAre;
    using Algorithm = PlaneSelectionAlgorithm::Algorithm;

    class KDTreeTest : public ::testing::TestWithParam<std::tuple<std::vector<Array3>, std::vector<IndexArray3>, std::vector<Array3>, Algorithm, bool>> {
    public:
        static const std::vector<Array3> cube_vertices;
        static const std::vector<IndexArray3> cube_faces;
        static const Polyhedron _big;

        static std::tuple<std::vector<Array3>, std::vector<IndexArray3>, std::vector<Array3>, Algorithm, bool> generateRandomPointsOnPolyhedron(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, const size_t n, Algorithm algorithm, bool paralellExecution) {
            std::vector<Array3> randomPoints;
            randomPoints.reserve(n);
            for (size_t i = 0; i < n; i++) {
                const auto faceIndex = getRandomIndex(faces.size() - 1);
                const auto &verticeIndices = faces.at(faceIndex);
                std::array<Array3, 3> faceVertices{};
                std::transform(verticeIndices.cbegin(), verticeIndices.cend(), faceVertices.begin(), [&](const auto index) { return vertices.at(index); });
                const auto point = randomPointOnFace(faceVertices);
                randomPoints.push_back(point);
            }
            return {vertices, faces, randomPoints, algorithm, paralellExecution};
        }

    protected:
        static constexpr unsigned long SEED = 8437529173464215;
        static constexpr double DELTA = 1e-8;
        static std::mt19937 gen;// mersenne_twister_engine seeded with SEED

        /**
         * Generates a random index between [0,sizeBuffer)
         * @param sizeBuffer Size of the buffer that the index should be applied to
         * @return the index
         */
        static int getRandomIndex(const int sizeBuffer) {
            std::uniform_int_distribution<> distrib(0, sizeBuffer - 1);
            return distrib(gen);
        }

        static Array3 randomPointOnFace(const std::array<Array3, 3> &vertices) {
            using namespace util;
            std::uniform_real_distribution<> distrib(0, 1);
            const double a{distrib(gen)};
            distrib = std::uniform_real_distribution<>(0, 1 - a);
            const double b{distrib(gen)};
            const double c{1 - a - b};

            return vertices[0] * a + vertices[1] * b + vertices[2] * c;
        }
    };

    const std::vector<Array3> KDTreeTest::cube_vertices{
            {-1.0, -1.0, -1.0},
            {1.0, -1.0, -1.0},
            {1.0, 1.0, -1.0},
            {-1.0, 1.0, -1.0},
            {-1.0, -1.0, 1.0},
            {1.0, -1.0, 1.0},
            {1.0, 1.0, 1.0},
            {-1.0, 1.0, 1.0}};
    const std::vector<IndexArray3> KDTreeTest::cube_faces{
            {1, 3, 2},
            {0, 3, 1},
            {0, 1, 5},
            {0, 5, 4},
            {0, 7, 3},
            {0, 4, 7},
            {1, 2, 6},
            {1, 6, 5},
            {2, 3, 6},
            {3, 7, 6},
            {4, 5, 6},
            {4, 6, 7}};

    std::mt19937 KDTreeTest::gen = std::mt19937(SEED);
    const Polyhedron KDTreeTest::_big{
            std::vector<std::string>{"resources/GravityModelBigTest.node", "resources/GravityModelBigTest.face"},
            1.0,
            NormalOrientation::OUTWARDS,
            PolyhedronIntegrity::DISABLE};


    TEST_P(KDTreeTest, PointsTest) {
        using namespace polyhedralGravity;
        using namespace util;
        const auto [vertices, faces, points, algorithm, paralellExecution] = GetParam();
        KDTree tree{vertices, faces, algorithm};
        constexpr Array3 origin{200, 200, 200};
        const auto pointTest = [&tree, &origin](const Array3 &point) {
                const auto ray{(point - origin) / 10.0};
                std::set<Array3> intersections;
                tree.getFaceIntersections(origin, ray, intersections);
                ASSERT_THAT(intersections, Contains(ElementsAre(DoubleNear(point[0], DELTA), DoubleNear(point[1], DELTA), DoubleNear(point[2], DELTA))));
        };
        if (paralellExecution) {
            thrust::for_each(thrust::host, points.cbegin(), points.cend(), pointTest);
        } else {
            std::for_each(points.cbegin(), points.cend(), pointTest);
        }
    }

    constexpr size_t numberOfPoints = 10;
    constexpr size_t bigNumberOfPoints = 1000;

    INSTANTIATE_TEST_SUITE_P(NoTreePointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), numberOfPoints, Algorithm::NOTREE, false)));
    INSTANTIATE_TEST_SUITE_P(QuadraticPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), numberOfPoints, Algorithm::QUADRATIC, false)));
    INSTANTIATE_TEST_SUITE_P(LogSquaredPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), numberOfPoints, Algorithm::LOGSQUARED, false)));
    INSTANTIATE_TEST_SUITE_P(LogPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), numberOfPoints, Algorithm::LOG,false)));

    INSTANTIATE_TEST_SUITE_P(NoTreePointsCube, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices, KDTreeTest::cube_faces, numberOfPoints, Algorithm::NOTREE, false)));
    INSTANTIATE_TEST_SUITE_P(QuadraticPointsCube, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices, KDTreeTest::cube_faces, numberOfPoints, Algorithm::QUADRATIC, false)));
    INSTANTIATE_TEST_SUITE_P(LogSquaredPointsCube, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices, KDTreeTest::cube_faces, numberOfPoints, Algorithm::LOGSQUARED, false)));
    INSTANTIATE_TEST_SUITE_P(LogPointsCube, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices, KDTreeTest::cube_faces, numberOfPoints, Algorithm::LOG, false)));

    INSTANTIATE_TEST_SUITE_P(NoTreeGreatNumberOfPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), bigNumberOfPoints, Algorithm::NOTREE, false)));
    INSTANTIATE_TEST_SUITE_P(LogSquaredGreatNumberOfPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), bigNumberOfPoints, Algorithm::LOGSQUARED, false)));
    INSTANTIATE_TEST_SUITE_P(LogGreatNumberOfPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), bigNumberOfPoints, Algorithm::LOG, false)));

    INSTANTIATE_TEST_SUITE_P(NoTreeGreatNumberOfPointsBigParalell, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), bigNumberOfPoints, Algorithm::NOTREE, true)));
    INSTANTIATE_TEST_SUITE_P(LogSquaredGreatNumberOfPointsBigParalell, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), bigNumberOfPoints, Algorithm::LOGSQUARED, true)));
    INSTANTIATE_TEST_SUITE_P(LogGreatNumberOfPointsBigParalell, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), bigNumberOfPoints, Algorithm::LOG, true)));

}// namespace polyhedralGravity