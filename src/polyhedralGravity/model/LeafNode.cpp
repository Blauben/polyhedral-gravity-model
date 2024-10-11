#include "LeafNode.h"

LeafNode::LeafNode(const SplitParam &splitParam)
    : TreeNode(splitParam) {
}

unsigned long LeafNode::countIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray) {
    return std::count_if(this->splitParam->indexBoundFaces.cbegin(), this->splitParam->indexBoundFaces.cend(), [this, ray, origin](const size_t faceIndex) {
        polyhedralGravity::Array3Triplet edgeVertices{};
        std::transform(this->splitParam->faces[faceIndex].cbegin(), this->splitParam->faces[faceIndex].cend(), edgeVertices.begin(), [this](const size_t vertexIndex) {
            return splitParam->vertices[vertexIndex];
        });
        return rayIntersectsTriangle(origin, ray, edgeVertices) != nullptr;
    });
}

void LeafNode::getFaceIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray, std::vector<size_t> &intersectedFaceIndices) {
    std::for_each(this->splitParam->indexBoundFaces.cbegin(), this->splitParam->indexBoundFaces.cend(), [this, &ray, &origin, &intersectedFaceIndices](const size_t faceIndex) {
        if (rayIntersectsTriangle(origin, ray, this->splitParam->faces[faceIndex]) != nullptr) {
            intersectedFaceIndices.push_back(faceIndex);
        }
    });
}

std::unique_ptr<polyhedralGravity::Array3> LeafNode::rayIntersectsTriangle(const polyhedralGravity::Array3 &rayOrigin, const polyhedralGravity::Array3 &rayVector, const polyhedralGravity::IndexArray3 &triangleVertexIndex) const {
    polyhedralGravity::Array3Triplet edgeVertices{};
    std::transform(triangleVertexIndex.cbegin(), triangleVertexIndex.cend(), edgeVertices.begin(), [this](const size_t vertexIndex) {
        return splitParam->vertices[vertexIndex];
    });
    return rayIntersectsTriangle(rayOrigin, rayVector, edgeVertices);
}

std::unique_ptr<polyhedralGravity::Array3> LeafNode::rayIntersectsTriangle(const polyhedralGravity::Array3 &rayOrigin, const polyhedralGravity::Array3 &rayVector, const polyhedralGravity::Array3Triplet &triangleVertices) {
    // Adapted Möller–Trumbore intersection algorithm
    // see https://en.wikipedia.org/wiki/Möller–Trumbore_intersection_algorithm
    using namespace polyhedralGravity;
    using namespace util;
    const Array3 edge1 = triangleVertices[1] - triangleVertices[0];
    const Array3 edge2 = triangleVertices[2] - triangleVertices[0];
    const Array3 h = cross(rayVector, edge2);
    const double a = util::dot(edge1, h);
    if (a > -EPSILON_ZERO_OFFSET && a < EPSILON_ZERO_OFFSET) {
        return nullptr;
    }

    const double f = 1.0 / a;
    const Array3 s = rayOrigin - triangleVertices[0];
    const double u = f * dot(s, h);
    if (u < 0.0 || u > 1.0) {
        return nullptr;
    }

    const Array3 q = cross(s, edge1);
    const double v = f * dot(rayVector, q);
    if (v < 0.0 || u + v > 1.0) {
        return nullptr;
    }

    const double t = f * dot(edge2, q);
    if (t > EPSILON_ZERO_OFFSET) {
        return std::make_unique<Array3>(rayOrigin + rayVector * t);
    } else {
        return nullptr;
    }
}