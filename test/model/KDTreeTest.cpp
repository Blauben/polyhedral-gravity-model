#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "polyhedralGravity/model/KDTree.h"
#include "polyhedralGravity/model/Polyhedron.h"
#include <array>

class KDTreeTest : public ::testing::Test {

protected:
    const std::vector<polyhedralGravity::Array3> _rectVertices{
            {-1.0, -1.0, -1.0},
            {1.0, -1.0, -1.0},
            {1.0, 1.0, -1.0},
            {-1.0, 1.0, -1.0},
            {-1.0, -1.0, 1.0},
            {1.0, -1.0, 1.0},
            {1.0, 1.0, 1.0},
            {-1.0, 1.0, 1.0},
            {0.0, -1.0, -1.0}};

    const std::vector<polyhedralGravity::IndexArray3> _facesOutwards{
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
            {4, 6, 7},
            {8, 0, 5}};
};

TEST_F(KDTreeTest, PrintPlane) {
    using namespace polyhedralGravity;
    using namespace testing;
    auto polyhedron = Polyhedron(_rectVertices, _facesOutwards, 1.0, NormalOrientation::OUTWARDS, PolyhedronIntegrity::HEAL);
    KDTree tree{};
    tree.buildKDTree(polyhedron);
    ASSERT_TRUE(tree.rootNode.has_value());
    EXPECT_EQ(tree.rootNode.value()->plane.first[0], 0);
}