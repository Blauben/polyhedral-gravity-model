#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "polyhedralGravity/model/KDTree.h"

#include "polyhedralGravity/input/TetgenAdapter.h"
#include "polyhedralGravity/model/Polyhedron.h"
#include <array>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace polyhedralGravity {
    class KDTreeTest : public ::testing::Test {
    public:
        /**
         * Generates a random index between [0,sizeBuffer)
         * @param sizeBuffer Size of the buffer that the index should be applied to
         * @return the index
         */
        static int getRandomIndex(int sizeBuffer) {
            return std::rand() % sizeBuffer;
        }

        static polyhedralGravity::Array3 randomPointOnFace(const std::array<polyhedralGravity::Array3, 3> &vertices) {
            using namespace polyhedralGravity::util;
            std::random_device rd; // a seed source for the random number engine
            std::mt19937 gen(rd());// mersenne_twister_engine seeded with rd()
            std::uniform_real_distribution<> distrib(0, 1);
            const double a{distrib(gen)};
            distrib = std::uniform_real_distribution<>(0, 1 - a);
            const double b{distrib(gen)};
            const double c{1 - a - b};

            return vertices[0] * a + vertices[1] * b + vertices[2] * c;
        }

        std::vector<polyhedralGravity::Array3> cube_vertices{
                {-1.0, -1.0, -1.0},
                {1.0, -1.0, -1.0},
                {1.0, 1.0, -1.0},
                {-1.0, 1.0, -1.0},
                {-1.0, -1.0, 1.0},
                {1.0, -1.0, 1.0},
                {1.0, 1.0, 1.0},
                {-1.0, 1.0, 1.0}};
        std::vector<polyhedralGravity::IndexArray3> cube_faces{
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
    };

    TEST_F(KDTreeTest, SinglePointCube) {
        using namespace polyhedralGravity::util;
        constexpr polyhedralGravity::Array3 point{0.5, 0, -1};
        constexpr int index{0};
        constexpr polyhedralGravity::Array3 origin{2, 0, 0};
        auto ray{(point - origin) / 10.0};
        KDTree tree{cube_vertices, cube_faces};
        std::vector<size_t> intersections;
        tree.getFaceIntersections(origin, ray, intersections);
        EXPECT_TRUE(std::find(intersections.cbegin(), intersections.cend(), index) != intersections.cend());
    }


    TEST_F(KDTreeTest, RandomSamplingCube) {
        using namespace polyhedralGravity;
        using namespace util;
        auto vertices = cube_vertices;
        auto faces = cube_faces;
        KDTree tree{vertices, faces};
        constexpr Array3 origin{2, 0, 0};
        for (int i{0}; i < 100; i++) {
            int index{getRandomIndex(static_cast<int>(faces.size()))};
            std::array<Array3, 3> faceVertices{};
            std::transform(faces[index].begin(), faces[index].end(), faceVertices.begin(), [vertices](const auto &vertexIndex) { return vertices[vertexIndex]; });
            auto point{randomPointOnFace(faceVertices)};
            auto ray{(point - origin) / 10.0};
            std::vector<size_t> intersections{};
            tree.getFaceIntersections(origin, ray, intersections);
            EXPECT_TRUE(std::find(intersections.cbegin(), intersections.cend(), index) != intersections.cend()) << "Point: " << point[0] << " " << point[1] << " " << point[2] << ", Index: " << index;
        }
    }

    TEST_F(KDTreeTest, SinglePointBig) {
        using namespace polyhedralGravity;
        using namespace util;
        TetgenAdapter src{std::vector<std::string>{"resources/GravityModelBigTest.node", "resources/GravityModelBigTest.face"}};
        auto [vertices, faces]{src.getPolyhedralSource()};
        constexpr Array3 point{0.63358522070580137, 0.28611476997259894, 0.071022829369024446};
        constexpr int index{7127};
        constexpr Array3 origin{200, 0, 0};
        auto ray{(point - origin) / 10.0};
        KDTree tree{vertices, faces};
        std::vector<size_t> intersections{};
        tree.getFaceIntersections(origin, ray, intersections);
        EXPECT_TRUE(std::find(intersections.cbegin(), intersections.cend(), index) != intersections.cend());
    }

    TEST_F(KDTreeTest, RandomSamplingBig) {
        using namespace polyhedralGravity;
        using namespace util;
        TetgenAdapter src{std::vector<std::string>{"resources/GravityModelBigTest.node", "resources/GravityModelBigTest.face"}};
        auto [vertices, faces]{src.getPolyhedralSource()};
        KDTree tree{vertices, faces};
        Array3 origin{0, 0, 0};
        for (int i{0}; i < 100; i++) {
            int index{getRandomIndex(static_cast<int>(faces.size()))};
            std::array<Array3, 3> faceVertices{};
            std::transform(faces[index].begin(), faces[index].end(), faceVertices.begin(), [vertices](const auto &vertexIndex) { return vertices[vertexIndex]; });
            auto point{randomPointOnFace(faceVertices)};
            auto ray{point - origin / 10};
            std::vector<size_t> intersections{};
            tree.getFaceIntersections(origin, ray, intersections);
            EXPECT_TRUE(std::find(intersections.cbegin(), intersections.cend(), index) != intersections.cend());
        }
    }
}
