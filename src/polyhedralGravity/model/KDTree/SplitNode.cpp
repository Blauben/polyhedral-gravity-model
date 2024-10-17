#include "polyhedralGravity/model/KDTree/SplitNode.h"

namespace polyhedralGravity {

    SplitNode::SplitNode(const SplitParam &splitParam, Plane &plane, TriangleIndexLists<2> &triangleIndexLists)
        : TreeNode(splitParam), _plane{plane}, _triangleIndexLists{std::move(triangleIndexLists)} {
        //convert the bound faces to its vertices
        std::vector<Array3> boundVertices{};
        boundVertices.reserve(splitParam.indexBoundFaces.size() * 3);
        std::for_each(splitParam.indexBoundFaces.cbegin(), splitParam.indexBoundFaces.cend(), [&boundVertices, &splitParam](const size_t faceIndex) {
            const auto vertices{KDTree::faceToVertices(splitParam.faces[faceIndex], splitParam.vertices)};
            boundVertices.insert(boundVertices.end(), vertices.cbegin(), vertices.cend());
        });
        _boundingBox = KDTree::getBoundingBox(boundVertices);
    }

    std::shared_ptr<TreeNode> SplitNode::getLesserNode() {
        if (!this->_lesser) {                                                                                                         //node is not yet built
            SplitParam childParam{*this->splitParam};                                                                                 //copy parent param and modify to fit new node
            childParam.boundingBox = (KDTree::splitBox(this->_boundingBox, this->_plane)).first;                                      //get the lesser box after splitting;
            childParam.indexBoundFaces = *std::move(_triangleIndexLists[0]);                                                          //get the triangles of the lesser box
            childParam.splitDirection = static_cast<Direction>((static_cast<int>(this->splitParam->splitDirection) + 1) % DIMENSIONS);//set the next splitting dimension: begin: (X -> Y -> Z); goto begin;
            _lesser = TreeNodeFactory::treeNodeFactory(childParam);
            maybeFreeParam();//if both nodes are built, split parameters are no longer needed
        }
        return _lesser;
    }

    std::shared_ptr<TreeNode> SplitNode::getGreaterNode() {
        if (!this->_greater) {                                                                 //node is not yet built
            SplitParam childParam{*this->splitParam};                                          //copy parent param and modify to fit new node
            childParam.boundingBox = KDTree::splitBox(this->_boundingBox, this->_plane).second;//get the greater box after splitting;
            childParam.indexBoundFaces = *std::move(_triangleIndexLists[1]);                   //get the triangles of the greater box
            childParam.splitDirection = static_cast<Direction>((static_cast<int>(this->splitParam->splitDirection) + 1) % DIMENSIONS);
            _greater = TreeNodeFactory::treeNodeFactory(childParam);
            maybeFreeParam();//if both nodes are built, split parameters are no longer needed
        }
        return _greater;
    }

    void SplitNode::maybeFreeParam() {
        if (_lesser && _greater) {//both nodes are built
            this->splitParam.reset();
        }
    }

    void SplitNode::getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) {
        const auto delegates = getChildrenForIntersection(origin, ray);                                         //calculate affected child nodes
        std::for_each(delegates.cbegin(), delegates.cend(), [&origin, &ray, &intersections](const auto &child) {//perform intersections on the child nodes
            child->getFaceIntersections(origin, ray, intersections);
        });
    }

    std::vector<std::shared_ptr<TreeNode>> SplitNode::getChildrenForIntersection(const Array3 &origin, const Array3 &ray) {
        using namespace polyhedralGravity::util;
        std::vector<std::shared_ptr<TreeNode>> delegates{};
        delegates.reserve(2);                                          //a SplitNode has max two children, so no more space needed.
        auto [t_enter, t_exit] = this->rayBoxIntersection(origin, ray);//calculate entry and exit points of the ray hitting the bounding box
        if (t_exit < t_enter || t_exit < 0) {                          // bounding box was not hit because the ray passed the box or is moving into the opposite direction of it, TODO: consider moving to root node because only called there
            return delegates;                                          //empty
        }
        const double t_split{rayPlaneIntersection(origin, ray)};                              //calculate point where plane was hit
        if (!std::isinf(t_split) && t_split >= 0 && t_enter <= t_split && t_exit >= t_split) {//the split plane is hit inside of the bounding box -> both child boxes need to be checked //TODO: consider optimizing: t_param stay the same in sub boxes
            delegates.push_back(getLesserNode());
            delegates.push_back(getGreaterNode());
        } else if (t_split < 0) {                                                                                                                                               // the origin is inside a box and the ray moves away from the plane
            delegates.push_back(origin[static_cast<int>(_plane.second)] < _plane.first ? getLesserNode() : getGreaterNode());                                                   //check in which point the origin lies in order to continue intersection in that box
        } else if (const double intersectionCoord{ray[static_cast<int>(_plane.second)] * t_enter + origin[static_cast<int>(_plane.second)]}; intersectionCoord < _plane.first) {// the entry point of the ray to the bounding box is nearer to the origin than the split plane -> ray hits lesser box
            delegates.push_back(getLesserNode());
        } else {// only the greater box is hit by the ray
            delegates.push_back(getGreaterNode());
        }
        return delegates;
    }

    std::pair<double, double> SplitNode::rayBoxIntersection(const Array3 &origin, const Array3 &ray) const {//refer to https://en.wikipedia.org/wiki/Slab_method
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