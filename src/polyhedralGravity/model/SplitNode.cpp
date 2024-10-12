#include "SplitNode.h"

#include "KDTree.h"
#include "KdDefinitions.h"
#include <algorithm>

SplitNode::SplitNode(const SplitParam &splitParam, Plane &plane, TriangleIndexLists<2> &triangleIndexLists)
    : TreeNode(splitParam), _plane{std::make_unique<Plane>(plane)}, _boundingBox{splitParam.boundingBox}, _triangleIndexLists{std::make_unique<TriangleIndexLists<2>>(std::move(triangleIndexLists))} {
}

TreeNode &SplitNode::getLesserNode() {
    if (!this->_lesser) {
        SplitParam childParam{*this->splitParam};
        childParam.boundingBox = (KDTree::splitBox(this->_boundingBox, *(this->_plane))).first;//first is the lesser box;
        childParam.indexBoundFaces = *(*_triangleIndexLists)[0];
        childParam.splitDirection = static_cast<Direction>((this->splitParam->splitDirection + 1) % DIMENSIONS);
        _lesser = treeNodeFactory(childParam);
        maybeFreeParam();
    }
    return *_lesser;
}

TreeNode &SplitNode::getGreaterNode() {
    if (!this->_greater) {
        SplitParam childParam{*this->splitParam};
        childParam.boundingBox = KDTree::splitBox(this->_boundingBox, *(this->_plane)).second;//second is the greater box
        childParam.indexBoundFaces = *(*_triangleIndexLists)[1];
        childParam.splitDirection = static_cast<Direction>((this->splitParam->splitDirection + 1) % DIMENSIONS);
        _greater = treeNodeFactory(childParam);
        maybeFreeParam();
    }
    return *_greater;
}

void SplitNode::maybeFreeParam() {
    if (_lesser && _greater) {
        this->splitParam.reset();
    }
}

unsigned long SplitNode::countIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray) {
    const auto delegates = getChildrenForIntersection(origin, ray);
    int intersectionCount{0};
    std::for_each(delegates.cbegin(), delegates.cend(), [&intersectionCount, &origin, &ray](const auto &child) {
        intersectionCount += child->countIntersections(origin, ray);
    });
    return intersectionCount;
}

void SplitNode::getFaceIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray, std::vector<size_t> &intersectedFaceIndices) {
    const auto delegates = getChildrenForIntersection(origin, ray);
    std::for_each(delegates.cbegin(), delegates.cend(), [&origin, &ray, &intersectedFaceIndices](const auto &child) {
        child->getFaceIntersections(origin, ray, intersectedFaceIndices);
    });
}

std::vector<TreeNode *> SplitNode::getChildrenForIntersection(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray) {
    using namespace polyhedralGravity::util;
    std::vector<TreeNode*> delegates{};
    delegates.reserve(2);
    auto [t_enter, t_exit] = this->rayBoxIntersection(origin, ray);
    if (t_exit < t_enter) {// bounding box was not hit, TODO: consider moving to root node because only called there
        return delegates;
    }
    if (double t_split{rayPlaneIntersection(origin, ray)}; t_enter < t_split && t_exit > t_split) { //TODO: consider optimizing: t_param stay the same in sub boxes
        delegates.push_back(&getLesserNode());
        delegates.push_back(&getGreaterNode());
    } else if (const double intersectionCoord{ray[_plane->second] * t_enter + origin[_plane->second]}; intersectionCoord < _plane->first) {
        delegates.push_back(&getLesserNode());
    } else {
        delegates.push_back(&getGreaterNode());
    }
    return delegates;
}

std::pair<double, double> SplitNode::rayBoxIntersection(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray) const {
    const double tx_min{(this->_boundingBox.first[0] - origin[0]) / ray[0]};
    const double ty_min{(this->_boundingBox.first[1] - origin[1]) / ray[1]};
    const double tz_min{(this->_boundingBox.first[2] - origin[2]) / ray[2]};
    const double tx_max{(this->_boundingBox.second[0] - origin[0]) / ray[0]};
    const double ty_max{(this->_boundingBox.second[1] - origin[1]) / ray[1]};
    const double tz_max{(this->_boundingBox.second[2] - origin[2]) / ray[2]};

    const double tx_enter{std::min(tx_min, tx_max)};
    const double tx_exit{std::max(tx_min, tx_max)};
    const double ty_enter{std::min(ty_min, ty_max)};
    const double ty_exit{std::max(ty_min, ty_max)};
    const double tz_enter{std::min(tz_min, tz_max)};
    const double tz_exit{std::max(tz_min, tz_max)};

    const double t_enter{std::max(tx_enter, std::max(ty_enter, tz_enter))};
    const double t_exit{std::min(tx_exit, std::min(ty_exit, tz_exit))};

    return std::make_pair(t_enter, t_exit);
}

double SplitNode::rayPlaneIntersection(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray) const {//Check for NaN in case of parallel plane and ray
    return (this->_plane->first - origin[this->_plane->second]) / ray[this->_plane->second];
}
