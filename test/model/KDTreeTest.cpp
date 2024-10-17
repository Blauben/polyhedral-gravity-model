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
    public:
        /**
         * Generates a random index between [0,sizeBuffer)
         * @param sizeBuffer Size of the buffer that the index should be applied to
         * @return the index
         */
        static int getRandomIndex(const int sizeBuffer) {
            return std::rand() % sizeBuffer;
        }

        static Array3 randomPointOnFace(const std::array<Array3, 3> &vertices) {
            using namespace util;
            std::random_device rd; // a seed source for the random number engine
            std::mt19937 gen(rd());// mersenne_twister_engine seeded with rd()
            std::uniform_real_distribution<> distrib(0, 1);
            const double a{distrib(gen)};
            distrib = std::uniform_real_distribution<>(0, 1 - a);
            const double b{distrib(gen)};
            const double c{1 - a - b};

            return vertices[0] * a + vertices[1] * b + vertices[2] * c;
        }

        template<typename T>
        static std::vector<T> toVector(std::set<T> set) {
            std::vector<T> result;
            std::for_each(set.cbegin(), set.cend(), [&result](T t) { result.push_back(t); });
            return result;
        }

        static bool isEqual(Array3 c1, Array3 c2) {
            if (c1.size() != c2.size()) return false;
            for (size_t i = 0; i < c1.size(); i++) {
                if (std::abs(c1[i] - c2[i]) > 1e-09) {
                    return false;
                }
            }
            return true;
        }

        std::vector<Array3> cube_vertices{
                {-1.0, -1.0, -1.0},
                {1.0, -1.0, -1.0},
                {1.0, 1.0, -1.0},
                {-1.0, 1.0, -1.0},
                {-1.0, -1.0, 1.0},
                {1.0, -1.0, 1.0},
                {1.0, 1.0, 1.0},
                {-1.0, 1.0, 1.0}};
        std::vector<IndexArray3> cube_faces{
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
        using namespace util;
        constexpr Array3 point{0.5, 0, -1};
        constexpr Array3 origin{2, 0, 0};
        auto ray{(point - origin) / 10.0};
        KDTree tree{cube_vertices, cube_faces};
        std::set<Array3> intersections;
        tree.getFaceIntersections(origin, ray, intersections);
        auto debug = toVector(intersections);
        EXPECT_TRUE(std::any_of(intersections.cbegin(), intersections.cend(), [&point](const auto &intersection) { return isEqual(point, intersection); })) << point[0] << " , " << point[1] << " , " << point[2];
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
            std::set<Array3> intersections;
            tree.getFaceIntersections(origin, ray, intersections);
            auto debug = toVector(intersections);
            EXPECT_TRUE(std::any_of(intersections.cbegin(), intersections.cend(), [&point](const auto &intersection) { return isEqual(point, intersection); })) << point[0] << " , " << point[1] << " , " << point[2];
        }
    }

    TEST_F(KDTreeTest, SinglePointBig) {
        using namespace polyhedralGravity;
        using namespace util;
        TetgenAdapter src{std::vector<std::string>{"resources/GravityModelBigTest.node", "resources/GravityModelBigTest.face"}};
        auto [vertices, faces]{src.getPolyhedralSource()};
        constexpr Array3 point{-0.3686961575432427, 0.070645976854416037, 0.26454502782828748};
        constexpr int index{7127};
        constexpr Array3 origin{200, 0, 0};
        auto ray{(point - origin) / 10.0};
        KDTree tree{vertices, faces};
        std::set<Array3> intersections;
        tree.getFaceIntersections(origin, ray, intersections);
        auto debug = toVector(intersections);
        EXPECT_TRUE(std::any_of(intersections.cbegin(), intersections.cend(), [&point](const auto &intersection) { return isEqual(point, intersection); })) << point[0] << " , " << point[1] << " , " << point[2];
    }

    TEST_F(KDTreeTest, Debug) {//FAIL: Index 399, point {0.23029106354915743, 0.15073537447485788, 0.15459847553287728}
        using namespace polyhedralGravity;
        using namespace util;
        TetgenAdapter src{std::vector<std::string>{"resources/GravityModelBigTest.node", "resources/GravityModelBigTest.face"}};
        auto [vertices, faces]{src.getPolyhedralSource()};
        constexpr Array3 point{0.23029106354915743, 0.15073537447485788, 0.15459847553287728};
        //-0.69817106185554101 , 0.22928626607740366 , -0.11489507670806454 Index: 10100
        constexpr int index{399};
        constexpr Array3 origin{0, 0, 0};
        auto ray{(point - origin) / 10.0};
        KDTree tree{vertices, faces};
        std::set<Array3> intersections;
        tree.getFaceIntersections(origin, ray, intersections);
        auto debug = toVector(intersections);
        EXPECT_TRUE(std::any_of(intersections.cbegin(), intersections.cend(), [&point](const auto &intersection) { return isEqual(point, intersection); })) << point[0] << " , " << point[1] << " , " << point[2];
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
            auto ray{(point - origin) / 10.0};
            std::set<Array3> intersections;
            tree.getFaceIntersections(origin, ray, intersections);
            auto debug = toVector(intersections);
            EXPECT_TRUE(std::any_of(intersections.cbegin(), intersections.cend(), [&point](const auto &intersection) { return isEqual(point, intersection); })) << point[0] << " , " << point[1] << " , " << point[2] << " Index: " << index;
        }
    }
}// namespace polyhedralGravity
