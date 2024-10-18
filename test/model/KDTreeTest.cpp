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
    class KDTreeTest : public ::testing::Test {
    protected:
        static constexpr unsigned long SEED = 8437529173464215;
        static constexpr double DELTA = 1e-9;
        std::mt19937 gen = std::mt19937(SEED);// mersenne_twister_engine seeded with SEED

        /**
         * Generates a random index between [0,sizeBuffer)
         * @param sizeBuffer Size of the buffer that the index should be applied to
         * @return the index
         */
        int getRandomIndex(const int sizeBuffer) {
            std::uniform_int_distribution<> distrib(0, sizeBuffer - 1);
            return distrib(gen);
        }

        Array3 randomPointOnFace(const std::array<Array3, 3> &vertices) {
            using namespace util;
            std::uniform_real_distribution<> distrib(0, 1);
            const double a{distrib(gen)};
            distrib = std::uniform_real_distribution<>(0, 1 - a);
            const double b{distrib(gen)};
            const double c{1 - a - b};

            return vertices[0] * a + vertices[1] * b + vertices[2] * c;
        }

        const std::vector<Array3> cube_vertices{
                {-1.0, -1.0, -1.0},
                {1.0, -1.0, -1.0},
                {1.0, 1.0, -1.0},
                {-1.0, 1.0, -1.0},
                {-1.0, -1.0, 1.0},
                {1.0, -1.0, 1.0},
                {1.0, 1.0, 1.0},
                {-1.0, 1.0, 1.0}};
        const std::vector<IndexArray3> cube_faces{
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

        Polyhedron _big{
                std::vector<std::string>{"resources/GravityModelBigTest.node", "resources/GravityModelBigTest.face"},
                1.0,
                polyhedralGravity::NormalOrientation::OUTWARDS,
                polyhedralGravity::PolyhedronIntegrity::DISABLE};

        static void singlePointTest(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, const Array3 &point);
        void randomPointTest(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces);
    };

    void KDTreeTest::singlePointTest(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, const Array3 &point) {
        using namespace util;
        constexpr Array3 origin{2, 0, 0};
        const auto ray{(point - origin) / 10.0};
        KDTree tree{vertices, faces};
        std::set<Array3> intersections;
        tree.getFaceIntersections(origin, ray, intersections);
        using testing::Contains;
        using testing::DoubleNear;
        using testing::ElementsAre;
        ASSERT_THAT(intersections, Contains(ElementsAre(DoubleNear(point[0], DELTA), DoubleNear(point[1], DELTA), DoubleNear(point[2], DELTA))));
    }

    TEST_F(KDTreeTest, SinglePointCube) {
        constexpr Array3 point{0.5, 0, -1};
        singlePointTest(cube_vertices, cube_faces, point);
    }

    TEST_F(KDTreeTest, SinglePointBig) {
        constexpr Array3 point{-0.3686961575432427, 0.070645976854416037, 0.26454502782828748};
        singlePointTest(_big.getVertices(), _big.getFaces(), point);
    }

    void KDTreeTest::randomPointTest(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces) {
        using namespace polyhedralGravity;
        using namespace util;
        KDTree tree{vertices, faces};
        constexpr Array3 origin{200, 200, 200};
        for (int i{0}; i < 10000; i++) {
            const int index{getRandomIndex(static_cast<int>(faces.size()))};
            std::array<Array3, 3> faceVertices{};
            std::transform(faces[index].begin(), faces[index].end(), faceVertices.begin(), [vertices](const auto &vertexIndex) { return vertices[vertexIndex]; });
            const auto point{randomPointOnFace(faceVertices)};
            const auto ray{(point - origin) / 10.0};
            std::set<Array3> intersections;
            tree.getFaceIntersections(origin, ray, intersections);
            using testing::Contains;
            using testing::DoubleNear;
            using testing::ElementsAre;
            ASSERT_THAT(intersections, Contains(ElementsAre(DoubleNear(point[0], DELTA), DoubleNear(point[1], DELTA), DoubleNear(point[2], DELTA))));
        }
    }


    TEST_F(KDTreeTest, RandomSamplingCube) {
        randomPointTest(cube_vertices, cube_faces);
    }

    TEST_F(KDTreeTest, RandomSamplingBig) {
        randomPointTest(_big.getVertices(), _big.getFaces());
    }
}// namespace polyhedralGravity
