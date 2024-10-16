#include "polyhedralGravity/model/KDTree/TreeNodeFactory.h"

namespace polyhedralGravity {
    std::unique_ptr<TreeNode> TreeNodeFactory::treeNodeFactory(const SplitParam &splitParam) {
        auto [plane, planeCost, triangleIndexLists] = KDTree::findPlane(splitParam);                                //find optimal plane splitting this node's bounding box
        if (planeCost > static_cast<double>(splitParam.indexBoundFaces.size()) * KDTree::triangleIntersectionCost) {//if the cost of splitting this node further is greater than just traversing the bound triangles, then don't split and return a LeafNode
            return std::make_unique<LeafNode>(splitParam);
        }
        return std::make_unique<SplitNode>(splitParam, plane, triangleIndexLists);//if not more costly, perform the split
    }
}// namespace polyhedralGravity