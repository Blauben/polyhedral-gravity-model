#include "SplitNode.h"

#include "KDTree.h"
#include "KdDefinitions.h"
#include "TreeNodeFactory.h"

#include <algorithm>

namespace polyhedralGravity {

    SplitNode::SplitNode(const SplitParam &splitParam, Plane &plane, TriangleIndexLists<2> &triangleIndexLists)
        : TreeNode(splitParam), _plane{plane}, _boundingBox{splitParam.boundingBox}, _triangleIndexLists{std::move(triangleIndexLists)} {
    }

    std::shared_ptr<TreeNode> SplitNode::getLesserNode() {
        if (!this->_lesser) {
            SplitParam childParam{*this->splitParam};
            childParam.boundingBox = (KDTree::splitBox(this->_boundingBox, this->_plane)).first;//first is the lesser box;
            childParam.indexBoundFaces = *_triangleIndexLists[0];
            _triangleIndexLists[0].reset();
            childParam.splitDirection = static_cast<Direction>((static_cast<int>(this->splitParam->splitDirection) + 1) % DIMENSIONS);
            _lesser = TreeNodeFactory::treeNodeFactory(childParam);
            maybeFreeParam();
        }
        return _lesser;
    }

    std::shared_ptr<TreeNode> SplitNode::getGreaterNode() {
        if (!this->_greater) {
            SplitParam childParam{*this->splitParam};
            childParam.boundingBox = KDTree::splitBox(this->_boundingBox, this->_plane).second;//second is the greater box
            childParam.indexBoundFaces = *_triangleIndexLists[1];
            _triangleIndexLists[1].reset();
            childParam.splitDirection = static_cast<Direction>((static_cast<int>(this->splitParam->splitDirection) + 1) % DIMENSIONS);
            _greater = TreeNodeFactory::treeNodeFactory(childParam);
            maybeFreeParam();
        }
        return _greater;
    }

    void SplitNode::maybeFreeParam() {
        if (_lesser && _greater) {
            this->splitParam.reset();
        }
    }

    void SplitNode::getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) {
        const auto delegates = getChildrenForIntersection(origin, ray);
        std::for_each(delegates.cbegin(), delegates.cend(), [&origin, &ray, &intersections](const auto &child) {
            child->getFaceIntersections(origin, ray, intersections);
        });
    }

    std::vector<std::shared_ptr<TreeNode>> SplitNode::getChildrenForIntersection(const Array3 &origin, const Array3 &ray) {
        using namespace polyhedralGravity::util;
        std::vector<std::shared_ptr<TreeNode>> delegates{};
        delegates.reserve(2);
        auto [t_enter, t_exit] = this->rayBoxIntersection(origin, ray);
        if (t_exit < t_enter || t_exit < 0) {// bounding box was not hit, TODO: consider moving to root node because only called there
            return delegates;
        }
        const double t_split{rayPlaneIntersection(origin, ray)};
        if (t_split > 0 && t_enter < t_split && t_exit > t_split) {//the split plane is hit inside of the bounding box -> both child boxes need to be checked //TODO: consider optimizing: t_param stay the same in sub boxes
            delegates.push_back(getLesserNode());
            delegates.push_back(getGreaterNode());
        } else if (t_split < 0) {                                                                                                                                               // only move in positive direction
            delegates.push_back(origin[static_cast<int>(_plane.second)] < _plane.first ? getLesserNode() : getGreaterNode());                                                   //check in which point the origin lies in order to continue intersection in that box
        } else if (const double intersectionCoord{ray[static_cast<int>(_plane.second)] * t_enter + origin[static_cast<int>(_plane.second)]}; intersectionCoord < _plane.first) {// the entry point of the ray to the bounding box is nearer to the origin than the split plane -> ray hits lesser box
            delegates.push_back(getLesserNode());
        } else {// only the greater box is hit by the ray
            delegates.push_back(getGreaterNode());
        }
        return delegates;
    }

    std::pair<double, double> SplitNode::rayBoxIntersection(const Array3 &origin, const Array3 &ray) const {
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

    double SplitNode::rayPlaneIntersection(const Array3 &origin, const Array3 &ray) const {//Check for NaN in case of parallel plane and ray
        return (this->_plane.first - origin[static_cast<int>(this->_plane.second)]) / ray[static_cast<int>(this->_plane.second)];
    }

}// namespace polyhedralGravity