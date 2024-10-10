#include "LeafNode.h"

LeafNode::LeafNode(const SplitParam& splitParam) : TreeNode(splitParam) {}

unsigned long LeafNode::countIntersections(const polyhedralGravity::Array3& origin, const polyhedralGravity::Array3& ray) {
    return std::count_if(this->splitParam->indexBoundFaces.cbegin(), this->splitParam->indexBoundFaces.cend(), [this, ray, origin](const size_t faceIndex) {
        polyhedralGravity::Array3Triplet edgeVertices{};
        std::transform(this->splitParam->faces[faceIndex].cbegin(), this->splitParam->faces[faceIndex].cend(), edgeVertices.begin(), [this](const size_t vertexIndex) {
            return splitParam->vertices[vertexIndex];
        });
        return rayIntersectsTriangle(origin, ray, edgeVertices) != nullptr;
    });
}

std::unique_ptr<polyhedralGravity::Array3> LeafNode::rayIntersectsTriangle(const polyhedralGravity::Array3 &rayOrigin, const polyhedralGravity::Array3 &rayVector, const polyhedralGravity::Array3Triplet &triangle) {
    // Adapted Möller–Trumbore intersection algorithm
    // see https://en.wikipedia.org/wiki/Möller–Trumbore_intersection_algorithm
    using namespace polyhedralGravity;
    using namespace util;
    const Array3 edge1 = triangle[1] - triangle[0];
    const Array3 edge2 = triangle[2] - triangle[0];
    const Array3 h = cross(rayVector, edge2);
    const double a = util::dot(edge1, h);
    if (a > -EPSILON_ZERO_OFFSET && a < EPSILON_ZERO_OFFSET) {
        return nullptr;
    }

    const double f = 1.0 / a;
    const Array3 s = rayOrigin - triangle[0];
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