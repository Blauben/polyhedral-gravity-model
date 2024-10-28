#include "polyhedralGravity/model/KDTree/TreeNodeFactory.h"

namespace polyhedralGravity {
    std::unique_ptr<TreeNode> TreeNodeFactory::treeNodeFactory(const SplitParam &splitParam, size_t currentRecursionDepth) {
        //avoid splitting after certain tree depth
        if (currentRecursionDepth >= MAX_RECURSION_DEPTH) {
            return std::make_unique<LeafNode>(splitParam, currentRecursionDepth);
        }
        //find optimal plane splitting this node's bounding box
        auto [plane, planeCost, triangleIndexLists] = KDTree::findPlane(splitParam);
        const double costWithoutSplit = static_cast<double>(splitParam.indexBoundFaces.size()) * KDTree::triangleIntersectionCost;
        //if the cost of splitting this node further is greater than just traversing the bound triangles, then don't split and return a LeafNode
        if (planeCost > costWithoutSplit) {
            return std::make_unique<LeafNode>(splitParam, currentRecursionDepth);
        }
        //if not more costly, perform the split
        return std::make_unique<SplitNode>(splitParam, plane, triangleIndexLists, currentRecursionDepth);
    }
}// namespace polyhedralGravity