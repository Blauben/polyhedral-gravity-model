#include "polyhedralGravity/model/KDTree/SplitNode.h"

namespace polyhedralGravity {

    SplitNode::SplitNode(const SplitParam &splitParam, const Plane &plane, TriangleIndexLists<2> &triangleIndexLists, size_t currentRecursionDepth)
        : TreeNode(splitParam, currentRecursionDepth), _plane{plane}, _triangleIndexLists{std::move(triangleIndexLists)}, _boundingBox{splitParam.boundingBox} {
    }

    std::shared_ptr<TreeNode> SplitNode::getChildNode(const size_t index) {
        //create a reference to store the built node in
        std::shared_ptr<TreeNode> &node = index == LESSER ? _lesser : _greater;
        //node is not yet built
        if (node == nullptr) {
            //copy parent param and modify to fit new node
            SplitParam childParam{*this->_splitParam};
            //get the bounding box after splitting;
            auto [lesserBox, greaterBox] = this->_boundingBox.splitBox(this->_plane);
            childParam.boundingBox = index == 0 ? lesserBox : greaterBox;
            //get the triangles of the box
            childParam.indexBoundFaces = *std::move(_triangleIndexLists[index]);
            childParam.splitDirection = static_cast<Direction>((static_cast<int>(this->_splitParam->splitDirection) + 1) % DIMENSIONS);
            //increase the recursion depth of the direct child by 1
            node = TreeNodeFactory::treeNodeFactory(childParam, _recursionDepth + 1);
            if (_lesser != nullptr && _greater != nullptr) {
                _splitParam.reset();
            }
        }
        return node;
    }

    std::vector<std::shared_ptr<TreeNode>> SplitNode::getChildrenForIntersection(const Array3 &origin, const Array3 &ray) {
        using namespace polyhedralGravity::util;
        std::vector<std::shared_ptr<TreeNode>> delegates{};
        //a SplitNode has max two children, so no more space needed.
        delegates.reserve(2);
        //calculate entry and exit points of the ray hitting the bounding box
        auto [t_enter, t_exit] = _boundingBox.rayBoxIntersection(origin, ray);
        // bounding box was not hit because the ray passed the box or is moving into the opposite direction of it, TODO: consider moving to root node because only called there
        if (t_exit < t_enter || t_exit < 0) {
            //empty
            return delegates;
        }
        //calculate point where plane was hit
        const double t_split{rayPlaneIntersection(origin, ray)};
        //TODO: consider optimizing: t_param stay the same in sub boxes
        //the split plane is hit inside of the bounding box -> both child boxes need to be checked
        const bool isParallel = std::isinf(t_split);
        bool planeIsHitInsideBox = 0 <= t_split && t_enter <= t_split && t_split <= t_exit;
        if (!isParallel && planeIsHitInsideBox) {
            delegates.push_back(getChildNode(LESSER));
            delegates.push_back(getChildNode(GREATER));
        }
        // the split plane is behind the ray origin
        else if (t_split < 0) {
            //check in which point the origin lies in order to continue intersection in that box
            delegates.push_back(origin[static_cast<int>(_plane.orientation)] < _plane.axisCoordinate ? getChildNode(LESSER) : getChildNode(GREATER));
            return delegates;
        }
        //intersection point of the ray and the bounding box
        const double intersectionCoord{ray[static_cast<int>(_plane.orientation)] * t_enter + origin[static_cast<int>(_plane.orientation)]};
        // the entry point of the ray to the bounding box is nearer to the origin than the split plane -> ray hits lesser box
        if (intersectionCoord < _plane.axisCoordinate) {
            delegates.push_back(getChildNode(LESSER));
        }
        // only the greater box is hit by the ray
        else {
            delegates.push_back(getChildNode(GREATER));
        }
        return delegates;
    }

    double SplitNode::rayPlaneIntersection(const Array3 &origin, const Array3 &ray) const {
        return (this->_plane.axisCoordinate - origin[static_cast<int>(this->_plane.orientation)]) / ray[static_cast<int>(this->_plane.orientation)];
    }

}// namespace polyhedralGravity