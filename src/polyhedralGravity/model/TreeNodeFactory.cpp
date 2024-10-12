#include "TreeNodeFactory.h"
#include "KDTree.h"
#include "LeafNode.h"
#include "SplitNode.h"

namespace polyhedralGravity {
    std::unique_ptr<TreeNode> TreeNodeFactory::treeNodeFactory(const SplitParam &splitParam) {
        auto [plane, planeCost, triangleIndexLists] = KDTree::findPlane(splitParam);
        if(planeCost > static_cast<double>(splitParam.indexBoundFaces.size()) * KDTree::triangleIntersectionCost) {
            return std::make_unique<LeafNode>(splitParam);
        }
        return std::make_unique<SplitNode>(splitParam, plane, triangleIndexLists);
    }
}