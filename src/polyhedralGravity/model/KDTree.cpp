#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <utility>

#include "KDTree.h"

KDTree::KDTree(const std::vector<polyhedralGravity::Array3>& vertices, const std::vector<polyhedralGravity::IndexArray3>& faces) {
    std::vector<size_t> boundFaces(faces.size());
    int index{0};
    std::generate(boundFaces.begin(), boundFaces.end(), [&index]() { return index++; });
    const Box boundingBox{getBoundingBox(vertices)};
    this->param = std::make_unique<SplitParam>(vertices, faces, boundFaces, boundingBox, X);
}

KDTree::KDTree(KDTree&& other) noexcept {
    this->rootNode = std::move(other.rootNode);
    if(!this->rootNode) {
        this->param = std::move(other.param);
    }
}

KDTree& KDTree::operator=(KDTree&& other) noexcept {
    if(*this == other) {
        return *this;
    }
    this->rootNode = std::move(other.rootNode);
    if(!this->rootNode) {
        this->param = std::move(other.param);
    }
    return *this;
}

bool KDTree::operator==(const KDTree & other) const {
    return this->rootNode == other.rootNode;
}

TreeNode &KDTree::getRootNode() {
    if (!this->rootNode) {
        this->rootNode = TreeNode::treeNodeFactory(*this->param.release());
    }
    return *this->rootNode;
}

unsigned long KDTree::countIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray) {
    return this->getRootNode().countIntersections(origin, ray);
}

void KDTree::getFaceIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray, std::vector<size_t> &intersectedFaceIndices) {
    this->getRootNode().getFaceIntersections(origin, ray, intersectedFaceIndices);
}


std::tuple<Plane, double, TriangleIndexLists> KDTree::findPlane(const SplitParam &param) {// O(N^2) implementation
    double cost = std::numeric_limits<double>::infinity();
    Plane optPlane{0, param.splitDirection};
    TriangleIndexLists optTriangleIndexLists{};
    std::for_each(param.indexBoundFaces.cbegin(), param.indexBoundFaces.cend(), [&param, &optPlane, &cost, &optTriangleIndexLists](const size_t faceIndex) {
        const auto &face = param.faces[faceIndex];
        for (const auto &index: face) {
            Plane candidatePlane{param.vertices[index][param.splitDirection], param.splitDirection};
            if (auto [candidateCost, triangleIndexLists] = costForPlane(param, candidatePlane); candidateCost < cost) {
                cost = candidateCost;
                optPlane = candidatePlane;
                optTriangleIndexLists = std::move(triangleIndexLists);
            }
        }
    });
    return std::make_tuple(std::move(optPlane), cost, std::move(optTriangleIndexLists));
}

Box KDTree::getBoundingBox(const std::vector<polyhedralGravity::Array3> &vertices) {
    using namespace polyhedralGravity;
    if(vertices.empty()) {
        return {{0, 0, 0}, {0, 0, 0}};
    }
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
    box1.second[axis] = plane.first;
    box2.first[axis] = plane.first;
    return std::make_pair(box1, box2);
}

std::pair<const double, TriangleIndexLists> KDTree::costForPlane(const SplitParam &param, const Plane &plane) {
    //Checks if the split plane is one of the faces of the bounding box, if so the split is useless
    if (plane.first == param.boundingBox.first[plane.second] || plane.first == param.boundingBox.second[plane.second]) {
        return std::make_pair(std::numeric_limits<double>::infinity(), TriangleIndexLists{});
    }
    auto [box1, box2] = splitBox(param.boundingBox, plane);
    auto [lessT, greaterT, equalT] = containedTriangles(param, plane);
    const double surfaceAreaBounding = surfaceAreaOfBox(param.boundingBox);
    const double surfaceArea1 = surfaceAreaOfBox(box1);
    const double surfaceArea2 = surfaceAreaOfBox(box2);
    const double costLesser = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * (static_cast<double>(lessT->size() + equalT->size())) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>(greaterT->size()));
    const double costUpper = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * static_cast<double>(lessT->size()) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>((greaterT->size() + equalT->size())));
    const double minCost = std::min(costLesser, costUpper) * (lessT->empty() || greaterT->empty() ? 0.8 : 1);
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
            if (vertex[split.second] < split.first) {
                less = true;
            } else if (vertex[split.second] > split.first) {
                greater = true;
            } else if (vertex[split.second] == split.first) {
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