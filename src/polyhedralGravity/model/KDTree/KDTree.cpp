#include "polyhedralGravity/model/KDTree/KDTree.h"

namespace polyhedralGravity {

    //on initialization of the tree a single bounding box which includes all the faces of the polyhedron is generated. Both the list of included faces and the parameters of the box are written to the split parameters
    KDTree::KDTree(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, const PlaneSelectionAlgorithm::Algorithm algorithm)
        : _vertices{vertices}, _faces{faces}, _splitParam{std::make_unique<SplitParam>(_vertices, _faces, Box::getBoundingBox(_vertices), Direction::X, PlaneSelectionAlgorithmFactory::create(algorithm))} {
    }

    std::shared_ptr<TreeNode> KDTree::getRootNode() {
        //if the node has already been generated, don't do it again. Instead let the factory determine the TreeNode subclass based on the optimal split.
        std::call_once(rootNodeCreated, [this]() {
            this->_rootNode = TreeNodeFactory::createTreeNode(*std::move(_splitParam), 0);
        });
        return this->_rootNode;
    }

    size_t KDTree::countIntersections(const Array3 &origin, const Array3 &ray) {
        //it's possible that a single intersection point is on the edge between two triangles. The point would be counted twice if the intersection points were not documented -> use of std::set
        std::set<Array3> set{};
        this->getFaceIntersections(origin, ray, set);
        return set.size();
    }

    void KDTree::getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) {
        //iterative approach to avoid stack and heap overflows
        //queue for children of processed nodes
        std::deque<std::shared_ptr<TreeNode>> queue{};
        //calculate inverse ray direction
        const Array3 inverseRay{1./ray[0], 1./ray[1], 1./ray[2]};
        //init with tree root
        queue.push_back(getRootNode());
        while (!queue.empty()) {
            auto node = queue.front();
            //if node is SplitNode perform intersection checks on the children and queue them accordingly
            if (const auto split = std::dynamic_pointer_cast<SplitNode>(node)) {
                const auto children = split->getChildrenForIntersection(origin, ray, inverseRay);
                thrust::for_each(thrust::host, std::begin(children), std::end(children), [&queue](const auto& child) { queue.push_back(child); });
            }
            //if node is leaf then perform intersections with the triangles contained
            else if (const auto leaf = std::dynamic_pointer_cast<LeafNode>(node)) {
                leaf->getFaceIntersections(origin, ray, intersections);
            }
            queue.pop_front();
        }
    }
}// namespace polyhedralGravity