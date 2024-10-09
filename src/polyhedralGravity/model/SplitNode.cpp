#include "SplitNode.h"

#include "KDTree.h"

SplitNode::SplitNode(SplitParam splitParam, Plane& plane, TriangleIndexLists& triangleIndexLists)
    : TreeNode(splitParam), _plane{std::make_unique<Plane>(plane)}, _triangleIndexLists{std::make_unique<TriangleIndexLists>(std::move(triangleIndexLists))} {
}

TreeNode* SplitNode::getLesserNode() {
    if (!this->_lesser) {
        SplitParam childParam{*this->splitParam};
        childParam.boundingBox = KDTree::splitBox(this->splitParam->boundingBox, *(this->_plane)).first; //first is the lesser box
        childParam.indexBoundFaces = *(*_triangleIndexLists)[0];
        childParam.indexBoundFaces.insert(childParam.indexBoundFaces.cend(), (*_triangleIndexLists)[2]->cbegin(), (*_triangleIndexLists)[2]->cend());
        childParam.splitDirection = static_cast<Direction>((this->splitParam->splitDirection + 1) % DIMENSIONS);
        _lesser = treeNodeFactory(childParam);
        maybeFreeParam();
    }
    return *(this->_lesser);
}

TreeNode* SplitNode::getGreaterNode() {
    if (!this->_greater) {
        SplitParam childParam{*this->splitParam};
        childParam.boundingBox = KDTree::splitBox(this->splitParam->boundingBox, *(this->_plane)).second; //second is the greater box
        childParam.indexBoundFaces = *(*_triangleIndexLists)[1];
        childParam.indexBoundFaces.insert(childParam.indexBoundFaces.cend(), (*_triangleIndexLists)[2]->cbegin(), (*_triangleIndexLists)[2]->cend());
        childParam.splitDirection = static_cast<Direction>((this->splitParam->splitDirection + 1) % DIMENSIONS);
        _greater = treeNodeFactory(childParam);
        maybeFreeParam();
    }
    return *(this->_greater);
}

void SplitNode::maybeFreeParam() {
    if(_lesser && _greater) {
        this->splitParam.reset();
    }
}

double SplitNode::intersect() {
    return 1.0;
}

