#include <optional>
#include <algorithm>
#include <limits>
#include <array>

#include "KDTree.h"
#include "KdDefinitions.h"

void KDTree::buildKDTree(const Polyhedron polyhedron) {
    std::vector<size_t> boundFaces(polyhedron.getFaces().size());
    int index{1};
    std::generate(boundFaces.begin(), boundFaces.end(), [&index]() {return index++;});
    const Box boundingBox{getBoundingBox(polyhedron.getVertices())};
    const SplitParam param{
            .vertices = polyhedron.getVertices(),
            .faces = polyhedron.getFaces(),
            .indexBoundFaces = boundFaces,
            .boundingBox = boundingBox,
            .splitDirection = Direction::X
    };
    KDTree::_rootNode = std::optional<std::unique_ptr<TreeNode>>(buildRectangle(param));
}

std::unique_ptr<TreeNode> KDTree::buildRectangle(const SplitParam& param) const {
    Plane plane = findPlane(param);
    return std::unique_ptr<TreeNode>{};
}

const Plane KDTree::findPlane(const SplitParam& param) const { // O(N^2) implementation
    double cost = std::numeric_limits<double>::infinity();
    Plane optPlane{std::array<double, 3>{0, 0, 0}, param.splitDirection};
    std::for_each(param.faces.cbegin(), param.faces.cend(), [&param, &optPlane, &cost, this](IndexArray3 face) {
        for(const auto& index : face) {
            Plane candidatePlane{param.vertices[index], param.splitDirection};
            const double candiateCost{costForPlane(param, candidatePlane)}; //TODO
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
    const Direction& axis{plane.second};
    box1.second[axis] = plane.first[axis];
    box2.first[axis] = plane.first[axis];
    return std::make_pair(box1, box2);
}

const double KDTree::costForPlane(const SplitParam& param, const Plane& plane) const {
    auto [lessT, greaterT, equalT] = containedTriangles(param, plane);
	auto [box1, box2] = splitBox(param.boundingBox, plane);
    double volumeBounding = volumeOfBox(param.boundingBox);
    double volume1 = volumeOfBox(box1);
    double volume2 = volumeOfBox(box2);
    //TODO: continue here
}

const double KDTree::surfaceAreaHeuristic(const double boundingVolume, const double fragmentVolume) const {
    return fragmentVolume / boundingVolume;
}

const double KDTree::volumeOfBox(const Box& box) const {
    double volume{1};
    for(int i{0}; i < box.size(); i++) {
        volume *= box.second[i] - box.first[i];
	}
    return volume;
}

const std::array<std::vector<size_t>, 3> KDTree::containedTriangles(const SplitParam& param, const Plane& split) const {
	std::vector<size_t> index_less{};
    std::vector<size_t> index_greater{};
    std::vector<size_t> index_equal{};

    std::for_each(param.indexBoundFaces.cbegin(), param.indexBoundFaces.cend(), [&param, &split, &index_equal, &index_greater, &index_less](const size_t faceIndex) {
        const IndexArray3& face{param.faces[index]};
       bool less{false}, greater{false}, equal{false};
       std::array<Array3,3> vertices{};
       std::transform(face.cbegin(), face.cend(), vertices.begin(), [&param](const size_t vertexIndex) {return param.vertices[vertexIndex];});
       for(const Array3 vertex :  vertices) {
           if(vertex[split.second] < split.first[split.second]) {
               less |= true;
           } else if(vertex[split.second] > split.first[split.second]) {
               greater |= true;
           } else if(vertex[split.second] == split.first[split.second]) {
               equal |= true;
           }
       }

       if((less && greater) || equal) {
           index_equal.push_back(faceIndex);
           } else if(greater) {
               index_greater.push_back(faceIndex);
           } else if(less) {
               index_less.push_back(faceIndex);
           }
    });
    return std::array{index_less, index_greater, index_equal};
}