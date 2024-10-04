#include <optional>
#include <algorithm>
#include <limits>

#include "KDTree.h"
#include "KdDefinitions.h"

void KDTree::buildKDTree(const Polyhedron polyhedron) {
    std::vector<size_t> boundFaces(polyhedron.getFaces().size());
    int index{1};
    std::generate(boundFaces.begin(), boundFaces.end(), [&index]() {return index++;});
    const Box boundingBox{getBoundingBox(polyhedron.getVertices())};
    KDTree::_currentDirection = Direction::VERTICAL;
    KDTree::_rootNode = std::optional<std::unique_ptr<TreeNode>>(buildRectangle(polyhedron.getVertices(), polyhedron.getFaces(), boundFaces, boundingBox));
}

std::unique_ptr<TreeNode> KDTree::buildRectangle(const std::vector<Array3>& vertices, const std::vector<IndexArray3>& faces, const std::vector<size_t> indexBoundFaces, const Box boundingBox) const {
    Plane plane = findPlane(vertices, faces, indexBoundFaces, boundingBox);
    return std::unique_ptr<TreeNode>{};
}

const Plane KDTree::findPlane(const std::vector<Array3>& vertices, const std::vector<IndexArray3>& faces, const std::vector<size_t> indexBoundFaces, Box boundingBox) const { // O(N^2) implementation
    double cost = std::numeric_limits<double>::infinity();
    const Direction& currentDirection{this->_currentDirection};
    Plane optPlane{std::array<double, 3>{0, 0, 0}, currentDirection};
    std::for_each(faces.cbegin(), faces.cend(), [&vertices, &currentDirection, &optPlane, &cost](IndexArray3 face) {
        for(const auto& index : face) {
            Plane candidatePlane{vertices[index], currentDirection};
            const double candiateCost{}; //TODO
            if(candiateCost < cost) {
                cost = candiateCost;
                optPlane = candidatePlane;
            }
        }
    });
    return optPlane;
}

const Box KDTree::getBoundingBox(const std::vector<Array3>& vertices) const {
    assert(vertices.size() != 0);
    Array3 min = vertices[0];
    Array3 max = vertices[0];
    std::for_each(vertices.cbegin(), vertices.cend(), [&min, &max](Array3 vertex) {
        for(int i = 0; i < vertex.size(); i++) {
            min[i] = std::min(min[i], vertex[i]);
            max[i] = std::max(max[i], vertex[i]);
        }
    });
    return Box(min, max);
}

const std::pair<Box,Box> KDTree::splitBox(const Box& box, const Plane& plane) const {
    Box box1{box};
    Box box2{box};
    switch(plane.second) { //TODO: continue here
        case Direction::HORIZONTAL: {

                break;
            }
            case Direction::VERTICAL: {

            break;
                }
        }
        return std::make_pair(box1, box2);
}

const double KDTree::costForPlane(const Plane plane, const std::vector<Array3>& vertices, const std::vector<IndexArray3>& faces, const std::vector<size_t> indexBoundFaces, const Box boundingBox) const {

}

const double KDTree::surfaceAreaHeuristic(double boundingVolume, double fragmentVolume) const {
    return fragmentVolume / boundingVolume;
}

