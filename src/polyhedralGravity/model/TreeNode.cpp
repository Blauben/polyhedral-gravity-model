#include "TreeNode.h"
#include "KDTree.h"
#include "LeafNode.h"
#include "SplitNode.h"

TreeNode::TreeNode(const SplitParam& splitParam)
    : splitParam(std::make_unique<SplitParam>(splitParam)) {
}

std::unique_ptr<TreeNode*> TreeNode::treeNodeFactory(const SplitParam &param) {
auto [plane, planeCost, triangleIndexLists] = KDTree::findPlane(param);
if(planeCost > static_cast<double>(param.indexBoundFaces.size()) * KDTree::triangleIntersectionCost) {
    LeafNode leaf{param};
    return std::make_unique<TreeNode*>(dynamic_cast<TreeNode*>(&leaf));
}
    SplitNode splitNode{param, plane, triangleIndexLists};
    return std::make_unique<TreeNode*>(dynamic_cast<TreeNode*>(&splitNode));
}