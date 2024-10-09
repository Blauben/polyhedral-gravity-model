#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <utility>

#include "KDTree.h"
#include "KdDefinitions.h"

KDTree::KDTree(const polyhedralGravity::Polyhedron &polyhedron) {
    std::vector<size_t> boundFaces(polyhedron.getFaces().size());
    int index{0};
    std::generate(boundFaces.begin(), boundFaces.end(), [&index]() { return index++; });
    const Box boundingBox{getBoundingBox(polyhedron.getVertices())};
    this->param = std::make_unique<SplitParam>(polyhedron.getVertices(), polyhedron.getFaces(), boundFaces, boundingBox, X);
}

TreeNode &KDTree::getRootNode() {
    if (!this->rootNode) {
        this->rootNode = TreeNode::treeNodeFactory(*std::move(this->param));
    }
    return *this->rootNode;
}

std::pair<Plane, TriangleIndexLists> KDTree::findPlane(const SplitParam &param) {// O(N^2) implementation
    double cost = std::numeric_limits<double>::infinity();
    Plane optPlane{param.vertices[0], param.splitDirection};
    TriangleIndexLists optTriangleIndexLists{};
    std::for_each(param.indexBoundFaces.cbegin(), param.indexBoundFaces.cend(), [&param, &optPlane, &cost, &optTriangleIndexLists](const size_t faceIndex) {
        const auto &face = param.faces[faceIndex];
        for (const auto &index: face) {
            Plane candidatePlane{param.vertices[index], param.splitDirection};
            if (auto [candidateCost, triangleIndexLists] = costForPlane(param, candidatePlane); candidateCost < cost) {
                cost = candidateCost;
                optPlane = candidatePlane;
                optTriangleIndexLists = std::move(triangleIndexLists);
            }
        }
    });
    return std::make_pair(std::move(optPlane), std::move(optTriangleIndexLists));
}

Box KDTree::getBoundingBox(const std::vector<polyhedralGravity::Array3> &vertices) {
    using namespace polyhedralGravity;
    assert(!vertices.empty());
    Array3 min = vertices[0];
    Array3 max = vertices[0];
    std::for_each(vertices.cbegin(), vertices.cend(), [&min, &max](const Array3 &vertex) {
        for (int i = 0; i < vertex.size(); i++) {
            min[i] = std::min(min[i], vertex[i]);
            max[i] = std::max(max[i], vertex[i]);
        }
    });
    return {min, max};
}

std::pair<Box, Box> KDTree::splitBox(const Box &box, const Plane &plane) {
    Box box1{box};
    Box box2{box};
    const Direction &axis{plane.second};
    box1.second[axis] = plane.first[axis];
    box2.first[axis] = plane.first[axis];
    return std::make_pair(box1, box2);
}

std::pair<const double, TriangleIndexLists> KDTree::costForPlane(const SplitParam &param, const Plane &plane) {
    auto [lessT, greaterT, equalT] = containedTriangles(param, plane);
    auto [box1, box2] = splitBox(param.boundingBox, plane);
    const double surfaceAreaBounding = surfaceAreaOfBox(param.boundingBox);
    const double surfaceArea1 = surfaceAreaOfBox(box1);
    const double surfaceArea2 = surfaceAreaOfBox(box2);
    const double costLesser = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * (lessT->size() + equalT->size()) + (surfaceArea2 / surfaceAreaBounding) * greaterT->size());
    const double costUpper = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * lessT->size() + (surfaceArea2 / surfaceAreaBounding) * (greaterT->size() + equalT->size()));
    const double minCost = std::min(costLesser, costUpper);
    return std::make_pair(minCost, TriangleIndexLists{std::move(lessT), std::move(greaterT), std::move(equalT)});
}

double KDTree::surfaceAreaOfBox(const Box &box) {
    const double width = std::abs(box.second[0] - box.first[0]);
    const double length = std::abs(box.second[1] - box.first[1]);
    const double height = std::abs(box.second[2] - box.first[2]);
    return 2 * (width * length + width * height + length * height);
}

TriangleIndexLists KDTree::containedTriangles(const SplitParam &param, const Plane &split) {
    using namespace polyhedralGravity;
    auto index_less = std::make_unique<std::vector<size_t>>();
    auto index_greater = std::make_unique<std::vector<size_t>>();
    auto index_equal = std::make_unique<std::vector<size_t>>();

    std::for_each(param.indexBoundFaces.cbegin(), param.indexBoundFaces.cend(), [&param, &split, &index_equal, &index_greater, &index_less](const size_t faceIndex) {
        const IndexArray3 &face{param.faces[faceIndex]};
        bool less{false}, greater{false}, equal{false};
        std::array<Array3, 3> vertices{};
        std::transform(face.cbegin(), face.cend(), vertices.begin(), [&param](const size_t vertexIndex) { return param.vertices[vertexIndex]; });
        for (const Array3 vertex: vertices) {
            if (vertex[split.second] < split.first[split.second]) {
                less = true;
            } else if (vertex[split.second] > split.first[split.second]) {
                greater = true;
            } else if (vertex[split.second] == split.first[split.second]) {
                equal = true;
            }
        }

        if ((less && greater) || equal) {
            index_equal->push_back(faceIndex);
        }
        if (greater) {
            index_greater->push_back(faceIndex);
        }
        if (less) {
            index_less->push_back(faceIndex);
        }
    });
    return std::array{std::move(index_less), std::move(index_greater), std::move(index_equal)};
}