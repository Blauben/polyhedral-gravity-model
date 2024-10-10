#include "TreeNode.h"
#include "KDTree.h"
#include "LeafNode.h"
#include "SplitNode.h"

TreeNode::TreeNode(const SplitParam& splitParam)
    : splitParam(std::make_unique<SplitParam>(splitParam)) {
}

std::unique_ptr<TreeNode> TreeNode::treeNodeFactory(const SplitParam &splitParam) {
auto [plane, planeCost, triangleIndexLists] = KDTree::findPlane(splitParam);
if(planeCost > static_cast<double>(splitParam.indexBoundFaces.size()) * KDTree::triangleIntersectionCost) {
    return std::make_unique<LeafNode>(splitParam);
}
    return std::make_unique<SplitNode>(splitParam, plane, triangleIndexLists);
}