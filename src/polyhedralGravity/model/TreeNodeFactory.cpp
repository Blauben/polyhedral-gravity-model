#include "TreeNodeFactory.h"
#include "KDTree.h"
#include "LeafNode.h"
#include "SplitNode.h"

namespace polyhedralGravity {
    std::unique_ptr<TreeNode> TreeNodeFactory::treeNodeFactory(const SplitParam &splitParam) {
        //find optimal plane splitting this node's bounding box
        auto [plane, planeCost, triangleIndexLists] = KDTree::findPlane(splitParam);
        //if the cost of splitting this node further is greater than just traversing the bound triangles, then don't split and return a LeafNode
        if (planeCost > static_cast<double>(splitParam.indexBoundFaces.size()) * KDTree::triangleIntersectionCost) {
            return std::make_unique<LeafNode>(splitParam);
        }
        //if not more costly, perform the split
        return std::make_unique<SplitNode>(splitParam, plane, triangleIndexLists);
    }
}// namespace polyhedralGravity