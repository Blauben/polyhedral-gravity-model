#include "polyhedralGravity/model/KDTree/KDTree.h"

#include "polyhedralGravity/input/TetgenAdapter.h"
#include "polyhedralGravity/model/Polyhedron.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace polyhedralGravity {
    using testing::Contains;
    using testing::DoubleNear;
    using testing::ElementsAre;
    using Algorithm = PlaneSelectionAlgorithm::Algorithm;

    class KDTreeTest : public ::testing::TestWithParam<std::tuple<std::vector<Array3>, std::vector<IndexArray3>, Algorithm,
                std::vector<Array3>> > {
    public:
        static const std::vector<Array3> cube_vertices;
        static const std::vector<IndexArray3> cube_faces;
        static const Polyhedron _big;

        static std::tuple<std::vector<Array3>, std::vector<IndexArray3>, Algorithm, std::vector<Array3>>
        generateRandomPointsOnPolyhedron(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, Algorithm algorithm,
                                         const size_t n) {
            std::vector<Array3> randomPoints;
            randomPoints.reserve(n);
            for (size_t i = 0; i < n; i++) {
                const auto faceIndex = getRandomIndex(faces.size() - 1);
                const auto &verticeIndices = faces.at(faceIndex);
                std::array<Array3, 3> faceVertices{};
                std::transform(verticeIndices.cbegin(), verticeIndices.cend(), faceVertices.begin(),
                               [&](const auto index) { return vertices.at(index); });
                const auto point = randomPointOnFace(faceVertices);
                randomPoints.push_back(point);
            }
            return {vertices, faces,  algorithm, randomPoints};
        }

    protected:
        static constexpr unsigned long SEED = 8437529173464215;
        static constexpr double DELTA = 1e-8;
        static std::mt19937 gen; // mersenne_twister_engine seeded with SEED

        /**
         * Generates a random index between [0,sizeBuffer)
         * @param sizeBuffer Size of the buffer that the index should be applied to
         * @return the index
         */
        static int getRandomIndex(const size_t sizeBuffer) {
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
        {-1.0, 1.0, 1.0}
    };
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
        {4, 6, 7}
    };

    std::mt19937 KDTreeTest::gen = std::mt19937(SEED); // NOLINT(*-msc51-cpp), predictable sequence wanted
    const Polyhedron KDTreeTest::_big{
        std::vector<std::string>{"resources/GravityModelBigTest.node", "resources/GravityModelBigTest.face"},
        1.0,
        NormalOrientation::OUTWARDS,
        PolyhedronIntegrity::DISABLE
    };


    TEST_P(KDTreeTest, PointsTest) {
        using namespace polyhedralGravity;
        using namespace util;
        const auto [vertices, faces, algorithm, points] = GetParam();
        KDTree tree{vertices, faces, algorithm};
        constexpr Array3 origin{200, 200, 200};
        const auto pointTest = [&tree, &origin](const Array3 &point) {
            const auto ray{(point - origin) / 10.0};
            std::set<Array3> intersections;
            tree.getFaceIntersections(origin, ray, intersections);
            ASSERT_THAT(intersections,
                        Contains(ElementsAre(DoubleNear(point[0], DELTA), DoubleNear(point[1], DELTA), DoubleNear(point[
                            2], DELTA))));
        };
        std::for_each(points.cbegin(), points.cend(), pointTest);
    }

    TEST_P(KDTreeTest, AlgorithmRegressionTest) {
        using namespace polyhedralGravity;
        using namespace util;
        std::vector<Array3> vertices;
        std::vector<IndexArray3> faces;
        Algorithm algorithm;
        std::tie(vertices, faces, algorithm, std::ignore) = GetParam();
        KDTree tree{vertices, faces, algorithm};
        auto squaredAlgorithm = PlaneSelectionAlgorithmFactory::create(PlaneEventAlgorithm::Algorithm::QUADRATIC);
        std::deque<std::shared_ptr<TreeNode>> nodePtrQueue{};
        nodePtrQueue.push_back(tree.getRootNode());
        while (!nodePtrQueue.empty()) {
            if (auto splitNodePtr = std::dynamic_pointer_cast<SplitNode>(nodePtrQueue.front())) {
                auto param = *splitNodePtr->_splitParam;
                // The squared algorithm obtains plane orientations by round robin but the plane event algorithms evaluate
                // all orientations and the choose the best one -> squared algorithm needs to know the desired orientation
                param.splitDirection = splitNodePtr->_plane.orientation;
                //const auto optimalPlane = std::get<0>(squaredAlgorithm->findPlane(param));
                const auto [optimalPlane, cost, triangles] = squaredAlgorithm->findPlane(param);
                ASSERT_EQ(optimalPlane, splitNodePtr->_plane) << "Check failed for node with id: " << splitNodePtr->nodeId <<"; " << splitNodePtr->_plane << " != " << optimalPlane <<std::endl;
                nodePtrQueue.push_back(splitNodePtr->getChildNode(0));
                nodePtrQueue.push_back(splitNodePtr->getChildNode(1));
            }
            nodePtrQueue.pop_front();
        }
    }

    constexpr size_t numberOfPoints = 10;
    constexpr size_t bigNumberOfPoints = 1000;

    INSTANTIATE_TEST_SUITE_P(NoTreePointsBig, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices
                                 (), KDTreeTest::_big.getFaces(), Algorithm::NOTREE, numberOfPoints)));
    INSTANTIATE_TEST_SUITE_P(QuadraticPointsBig, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices
                                 (), KDTreeTest::_big.getFaces(), Algorithm::QUADRATIC, numberOfPoints)));
    INSTANTIATE_TEST_SUITE_P(LogSquaredPointsBig, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices
                                 (), KDTreeTest::_big.getFaces(), Algorithm::LOGSQUARED, numberOfPoints)));
    INSTANTIATE_TEST_SUITE_P(LogPointsBig, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices
                                 (), KDTreeTest::_big.getFaces(), Algorithm::LOG, numberOfPoints)));

    INSTANTIATE_TEST_SUITE_P(NoTreePointsCube, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices,
                                 KDTreeTest::cube_faces, Algorithm::NOTREE, numberOfPoints)));
    INSTANTIATE_TEST_SUITE_P(QuadraticPointsCube, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices,
                                 KDTreeTest::cube_faces, Algorithm::QUADRATIC, numberOfPoints)));
    INSTANTIATE_TEST_SUITE_P(LogSquaredPointsCube, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices,
                                 KDTreeTest::cube_faces, Algorithm::LOGSQUARED, numberOfPoints)));
    INSTANTIATE_TEST_SUITE_P(LogPointsCube, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices,
                                 KDTreeTest::cube_faces, Algorithm::LOG, numberOfPoints)));

    INSTANTIATE_TEST_SUITE_P(NoTreeGreatNumberOfPointsBig, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices
                                 (), KDTreeTest::_big.getFaces(), Algorithm::NOTREE, bigNumberOfPoints)));
    INSTANTIATE_TEST_SUITE_P(LogSquaredGreatNumberOfPointsBig, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices
                                 (), KDTreeTest::_big.getFaces(), Algorithm::LOGSQUARED, bigNumberOfPoints)));
    INSTANTIATE_TEST_SUITE_P(LogGreatNumberOfPointsBig, KDTreeTest,
                             ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices
                                 (), KDTreeTest::_big.getFaces(), Algorithm::LOG, bigNumberOfPoints)));
} // namespace polyhedralGravity
