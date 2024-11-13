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

    class KDTreeTest : public ::testing::TestWithParam<std::tuple<std::vector<Array3>, std::vector<IndexArray3>, std::vector<Array3>, Algorithm>> {
    public:
        static const std::vector<Array3> cube_vertices;
        static const std::vector<IndexArray3> cube_faces;
        static const Polyhedron _big;

        static std::tuple<std::vector<Array3>, std::vector<IndexArray3>, std::vector<Array3>, Algorithm> generateRandomPointsOnPolyhedron(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, const size_t n, Algorithm algorithm) {
            std::vector<Array3> randomPoints;
            randomPoints.reserve(n);
            for (size_t i = 0; i < n; i++) {
                const auto &verticeIndices = faces.at(getRandomIndex(faces.size() - 1));
                std::array<Array3, 3> faceVertices{};
                std::transform(verticeIndices.cbegin(), verticeIndices.cend(), faceVertices.begin(), [&](const auto index) { return vertices.at(index); });
                randomPoints.emplace_back(randomPointOnFace(faceVertices));
            }
            return {vertices, faces, randomPoints, algorithm};
        }

    protected:
        static constexpr unsigned long SEED = 8437529173464215;
        static constexpr double DELTA = 1e-9;
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
        const auto [vertices, faces, points, algorithm] = GetParam();
        KDTree tree{vertices, faces, algorithm};
        constexpr Array3 origin{200, 200, 200};
        for (const auto &point: points) {
            const auto ray{(point - origin) / 10.0};
            std::set<Array3> intersections;
            tree.getFaceIntersections(origin, ray, intersections);
            ASSERT_THAT(intersections, Contains(ElementsAre(DoubleNear(point[0], DELTA), DoubleNear(point[1], DELTA), DoubleNear(point[2], DELTA))));
        }
    }


    INSTANTIATE_TEST_SUITE_P(SinglePointCube, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices, KDTreeTest::cube_faces, 1, Algorithm::LOG)));
    INSTANTIATE_TEST_SUITE_P(RandomPointCube, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices, KDTreeTest::cube_faces, 10000, Algorithm::LOG)));
    INSTANTIATE_TEST_SUITE_P(SinglePointBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), 1, Algorithm::LOG)));
    INSTANTIATE_TEST_SUITE_P(RandomPointBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), 10000, Algorithm::LOG)));

    constexpr size_t numberOfPoints = 10;
    INSTANTIATE_TEST_SUITE_P(NoTreePointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), numberOfPoints, Algorithm::NOTREE)));
    INSTANTIATE_TEST_SUITE_P(QuadraticPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), numberOfPoints, Algorithm::QUADRATIC)));
    INSTANTIATE_TEST_SUITE_P(LogSquaredPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), numberOfPoints, Algorithm::LOGSQUARED)));
    INSTANTIATE_TEST_SUITE_P(LogPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), numberOfPoints, Algorithm::LOG)));

    INSTANTIATE_TEST_SUITE_P(NoTreePointsCube, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices, KDTreeTest::cube_faces, numberOfPoints, Algorithm::NOTREE)));
    INSTANTIATE_TEST_SUITE_P(QuadraticPointsCube, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices, KDTreeTest::cube_faces, numberOfPoints, Algorithm::QUADRATIC)));
    INSTANTIATE_TEST_SUITE_P(LogSquaredPointsCube, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices, KDTreeTest::cube_faces, numberOfPoints, Algorithm::LOGSQUARED)));
    INSTANTIATE_TEST_SUITE_P(LogPointsCube, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::cube_vertices, KDTreeTest::cube_faces, numberOfPoints, Algorithm::LOG)));

    constexpr size_t bigNumberOfPoints = 10000;
    INSTANTIATE_TEST_SUITE_P(NoTreeGreatNumberOfPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), bigNumberOfPoints, Algorithm::NOTREE)));
    INSTANTIATE_TEST_SUITE_P(LogSquaredGreatNumberOfPointsBig, KDTreeTest, ::testing::Values(KDTreeTest::generateRandomPointsOnPolyhedron(KDTreeTest::_big.getVertices(), KDTreeTest::_big.getFaces(), bigNumberOfPoints, Algorithm::LOGSQUARED)));


    /*
    TEST_F(KDTreeTest, debugLog) {//TODO: remove
        using namespace polyhedralGravity;
        using namespace util;
        KDTree treeLog{_big.getVertices(), _big.getFaces(), Algorithm::LOG};
        KDTree treeLogSquared{_big.getVertices(), _big.getFaces(), Algorithm::LOGSQUARED};
        auto variantLog = std::dynamic_pointer_cast<SplitNode>(treeLog.getRootNode())->getTriangleSplitSets();
        auto variantLogSquared = std::dynamic_pointer_cast<SplitNode>(treeLogSquared.getRootNode())->getTriangleSplitSets();
        ASSERT_TRUE(std::holds_alternative<PlaneEventLists<2>>(variantLog));
        ASSERT_TRUE(std::holds_alternative<TriangleIndexLists<2>>(variantLogSquared));
        const auto rootEventsLog3D = std::get<PlaneEventLists<2>>(std::move(variantLog));
        const auto rootTrianglesLogSquared = std::get<TriangleIndexLists<2>>(std::move(variantLogSquared));
        PlaneEventLists<2> rootEventsLogSquared{};
        std::transform(rootTrianglesLogSquared.cbegin(), rootTrianglesLogSquared.cend(), rootEventsLogSquared.begin(), [&treeLogSquared](const auto &list) {
            const SplitParam splitParam(_big.getVertices(), _big.getFaces(), *list, Box::getBoundingBox(_big.getVertices()).splitBox(Plane(0.059034641527489787, Direction::X)).first, Direction::Y, PlaneSelectionAlgorithmFactory::create(Algorithm::LOGSQUARED));
            return std::make_unique<PlaneEventList>(PlaneEventAlgorithm::generatePlaneEventsFromFaces(splitParam, {Direction::X, Direction::Y, Direction::Z}));
        });
        std::vector<PlaneEvent> event1294{};
        std::copy_if(rootEventsLog3D[0]->cbegin(), rootEventsLog3D[0]->cend(), std::back_inserter(event1294), [](const auto &event) { return event.faceIndex == 1294; });
        std::sort(event1294.begin(), event1294.end());
        //all plane events of logNSquared are contained in logN
        PlaneEventList errors{};
        for (const auto &planeEvent: *(rootEventsLogSquared[0])) {
            auto it = std::find(rootEventsLog3D[0]->cbegin(), rootEventsLog3D[0]->cend(), planeEvent);
            if (it == rootEventsLog3D[0]->cend()) {
                errors.push_back(planeEvent);
            }
        }
        ASSERT_EQ(errors.size(), 0);
        /*
        std::vector<PlaneEvent> result{};
        for (auto i = 0; i < 2; i++) {
            for (const PlaneEvent &event: *(rootEventsLogSquared[i])) {
                std::copy_if(root)
            }
        }
        ASSERT_EQ(rootEventsLog.size(), 0);
*/
    /*
        constexpr Array3 origin{200, 200, 200};
        const Array3 point{0.47696049197143608, 0.15726201114874008, 0.20146690694962635};
        const auto ray{(point - origin) / 10.0};
        std::set<Array3> intersections;
        tree.getFaceIntersections(origin, ray, intersections);
        tree.printTree();
        ASSERT_THAT(intersections, Contains(ElementsAre(DoubleNear(point[0], DELTA), DoubleNear(point[1], DELTA), DoubleNear(point[2], DELTA))));

    }
*/
}// namespace polyhedralGravity
