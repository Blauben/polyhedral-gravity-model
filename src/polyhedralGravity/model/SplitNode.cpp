#include "SplitNode.h"

#include "KDTree.h"

SplitNode::SplitNode(SplitParam splitParam)
    : TreeNode(std::make_unique<SplitParam>(splitParam)) {
}

TreeNode &SplitNode::getLesserNode() {
    if (!this->_lesser) {
        computeChildren();
    }
    return *(this->_lesser);
}

TreeNode &SplitNode::getGreaterNode() {
    if (!this->_greater) {
        computeChildren();
    }
    return *(this->_greater);
}

void SplitNode::computeChildren() {
    auto [plane, triangleIndexLists] = KDTree::findPlane(*splitParam);
    this->_plane = std::make_unique<Plane>(plane);
    auto [lesserBox, greaterBox] = KDTree::splitBox(splitParam->boundingBox, *_plane);
    splitParam->splitDirection = static_cast<Direction>((splitParam->splitDirection + 1) % DIMENSIONS);
    splitParam->indexBoundFaces = *triangleIndexLists[0];//TODO: optimization maybe use std::list
    splitParam->indexBoundFaces.insert(splitParam->indexBoundFaces.cend(), triangleIndexLists[2]->cbegin(), triangleIndexLists[2]->cend());
    splitParam->boundingBox = lesserBox;
    _lesser = treeNodeFactory(*splitParam);
    splitParam->indexBoundFaces = *triangleIndexLists[1];//TODO: optimization maybe use std::list
    splitParam->indexBoundFaces.insert(splitParam->indexBoundFaces.cend(), triangleIndexLists[2]->cbegin(), triangleIndexLists[2]->cend());
    splitParam->boundingBox = greaterBox;
    _greater = treeNodeFactory(*std::move(splitParam));
}
