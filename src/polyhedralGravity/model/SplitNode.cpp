#include "SplitNode.h"

#include "KDTree.h"
#include "KdDefinitions.h"
#include "TreeNodeFactory.h"

#include <algorithm>

namespace polyhedralGravity {

    SplitNode::SplitNode(const SplitParam &splitParam, Plane &plane, TriangleIndexLists<2> &triangleIndexLists)
        : TreeNode(splitParam), _plane{plane}, _triangleIndexLists{std::move(triangleIndexLists)} {
        //convert the bound faces to its vertices
        std::vector<Array3> boundVertices{};
        boundVertices.reserve(splitParam.indexBoundFaces.size() * 3);
        std::for_each(splitParam.indexBoundFaces.cbegin(), splitParam.indexBoundFaces.cend(), [&boundVertices, &splitParam](const size_t faceIndex) {
            for (const auto &vertexIndex: splitParam.faces[faceIndex]) {
                boundVertices.push_back(splitParam.vertices[vertexIndex]);
            }
        });
        _boundingBox = KDTree::getBoundingBox(boundVertices);
    }

    std::shared_ptr<TreeNode> SplitNode::getChildNode(size_t index) {
        std::shared_ptr<TreeNode> node = index == LESSER ? _lesser : _greater;
        //node is not yet built
        if(node == nullptr) {
            //copy parent param and modify to fit new node
            SplitParam childParam{*this->splitParam};
            //get the bounding box after splitting;
            auto [lesserBox, greaterBox] = KDTree::splitBox(this->_boundingBox, this->_plane);
            childParam.boundingBox = index == 0 ? lesserBox : greaterBox;
            //get the triangles of the box
            childParam.indexBoundFaces = *std::move(_triangleIndexLists[index]);
            childParam.splitDirection = static_cast<Direction>((static_cast<int>(this->splitParam->splitDirection) + 1) % DIMENSIONS);
            node = TreeNodeFactory::treeNodeFactory(childParam);
        }
    return node;
    }

    void SplitNode::getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) {
        //calculate affected child nodes
        const auto delegates = getChildrenForIntersection(origin, ray);
        //perform intersections on the child nodes
        std::for_each(delegates.cbegin(), delegates.cend(), [&origin, &ray, &intersections](const auto &child) {
            child->getFaceIntersections(origin, ray, intersections);
        });
    }

    std::vector<std::shared_ptr<TreeNode>> SplitNode::getChildrenForIntersection(const Array3 &origin, const Array3 &ray) {
        using namespace polyhedralGravity::util;
        std::vector<std::shared_ptr<TreeNode>> delegates{};
        //a SplitNode has max two children, so no more space needed.
        delegates.reserve(2);
        //calculate entry and exit points of the ray hitting the bounding box
        auto [t_enter, t_exit] = this->rayBoxIntersection(origin, ray);
        // bounding box was not hit because the ray passed the box or is moving into the opposite direction of it, TODO: consider moving to root node because only called there
        if (t_exit < t_enter || t_exit < 0) {
            //empty
            return delegates;
        }
        //calculate point where plane was hit
        const double t_split{rayPlaneIntersection(origin, ray)};
        //TODO: consider optimizing: t_param stay the same in sub boxes
        //the split plane is hit inside of the bounding box -> both child boxes need to be checked
        if (!std::isinf(t_split) && t_split >= 0 && t_enter <= t_split && t_exit >= t_split) {
            delegates.push_back(getChildNode(LESSER));
            delegates.push_back(getChildNode(GREATER));
        }
        // the split plane is behind the ray origin
        else if (t_split < 0) {
            //check in which point the origin lies in order to continue intersection in that box
            delegates.push_back(origin[static_cast<int>(_plane.second)] < _plane.first ? getChildNode(LESSER) : getChildNode(GREATER));
            return delegates;
        }
        //intersection point of the ray and the split plane
        const double intersectionCoord{ray[static_cast<int>(_plane.second)] * t_enter + origin[static_cast<int>(_plane.second)]};
        // the entry point of the ray to the bounding box is nearer to the origin than the split plane -> ray hits lesser box
        if (intersectionCoord < _plane.first) {
            delegates.push_back(getChildNode(LESSER));
        }
        // only the greater box is hit by the ray
        else {
            delegates.push_back(getChildNode(GREATER));
        }
        return delegates;
    }

    //en.wikipedia.org/wiki/Slab_method
    std::pair<double, double> SplitNode::rayBoxIntersection(const Array3 &origin, const Array3 &ray) const {//refer to https:
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

    double SplitNode::rayPlaneIntersection(const Array3 &origin, const Array3 &ray) const {
        return (this->_plane.first - origin[static_cast<int>(this->_plane.second)]) / ray[static_cast<int>(this->_plane.second)];
    }

}// namespace polyhedralGravity