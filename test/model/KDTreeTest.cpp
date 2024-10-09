#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "polyhedralGravity/model/KDTree.h"

#include "polyhedralGravity/input/TetgenAdapter.h"
#include "polyhedralGravity/model/Polyhedron.h"
#include <array>
#include <string>
#include <utility>
#include <vector>

class KDTreeTest : public ::testing::Test {

public:
    static Polyhedron constructPolyhedron(std::string plyFile) {
        std::vector<std::string> fileList{std::move(plyFile)};
        TetgenAdapter reader{fileList};
        auto [vertices, faces]{reader.getPolyhedralSource()};
        return {vertices, faces, 1.0};
    }

protected:
    const std::vector<polyhedralGravity::Array3> _rectVertices{
            {-1.0, -1.0, -1.0},
            {1.0, -1.0, -1.0},
            {1.0, 1.0, -1.0},
            {-1.0, 1.0, -1.0},
            {-1.0, -1.0, 1.0},
            {1.0, -1.0, 1.0},
            {1.0, 1.0, 1.0},
            {-1.0, 1.0, 1.0}};

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
    };
};

/**
 *  Tests whether the root split plane found by the KDTree build algorithm is anchored on one of the vertices
 *
*/
TEST_F(KDTreeTest, VertexOnPlane) {
    using namespace polyhedralGravity;
    using namespace testing;
    auto [rootNode]{KDTree::buildKDTree(Polyhedron(_rectVertices, _facesOutwards, 1.0, NormalOrientation::OUTWARDS, PolyhedronIntegrity::HEAL))};
    ASSERT_TRUE(rootNode);
    EXPECT_EQ(rootNode->plane.first[0], 1);
}

/**
 * Empty space should be cut off instead of halving in the middle
 */
TEST_F(KDTreeTest, SplitCube) {
    using namespace polyhedralGravity;
    using namespace testing;
    auto [rootNode]{KDTree::buildKDTree(constructPolyhedron("resources/CubeXDivided.ply"))};
    ASSERT_TRUE(rootNode);
    EXPECT_NE(rootNode->plane.first[0], 0);
}