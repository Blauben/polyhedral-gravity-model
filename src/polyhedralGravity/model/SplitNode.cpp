#include "SplitNode.h"

#include "KDTree.h"

SplitNode::SplitNode(const SplitParam& splitParam, Plane& plane, TriangleIndexLists& triangleIndexLists)
    : TreeNode(splitParam), _plane{std::make_unique<Plane>(plane)}, _triangleIndexLists{std::make_unique<TriangleIndexLists>(std::move(triangleIndexLists))} {
}

TreeNode* SplitNode::getLesserNode() {
    if (!this->_lesser) {
        SplitParam childParam{*this->splitParam};
        childParam.boundingBox = (KDTree::splitBox(this->splitParam->boundingBox, *(this->_plane))).first; //first is the lesser box;
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

unsigned long SplitNode::countIntersections(const polyhedralGravity::Array3& origin, const polyhedralGravity::Array3& ray) {
    return 1; //TODO
}

std::pair<polyhedralGravity::Array3, polyhedralGravity::Array3> SplitNode::rayBoxIntersection(const polyhedralGravity::Array3& origin, const polyhedralGravity::Array3& ray) {
    const double tx_min{(this->splitParam->boundingBox.first[0] - origin[0])/ray[0]};
    const double ty_min{(this->splitParam->boundingBox.first[1] - origin[1])/ray[1]};
    const double tz_min{(this->splitParam->boundingBox.first[2] - origin[2])/ray[2]};
    const double tx_max{(this->splitParam->boundingBox.second[0] - origin[0])/ray[0]};
    const double ty_max{(this->splitParam->boundingBox.second[1] - origin[1])/ray[1]};
    const double tz_max{(this->splitParam->boundingBox.second[2] - origin[2])/ray[2]};

    const double tx_enter{std::min(tx_min, tx_max)};
    const double tx_exit{std::max(tx_min, tx_max)};
    const double ty_enter{std::min(ty_min, ty_max)};
    const double ty_exit{std::max(ty_min, ty_max)};
    const double tz_enter{std::min(tz_min, tz_max)};
    const double tz_exit{std::max(tz_min, tz_max)};

    const double t_enter{std::max(tx_enter, ty_enter, tz_enter)};
    const double t_exit{std::min(tx_exit, ty_exit, tz_exit)};

    return std::make_pair(origin + ray * t_enter, origin + ray * t_exit);
}

polyhedralGravity::Array3 SplitNode::rayPlaneIntersection(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray) {
    return {0,0,0};
 //TODO
}


