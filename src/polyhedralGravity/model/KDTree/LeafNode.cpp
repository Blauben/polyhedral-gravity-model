#include "polyhedralGravity/model/KDTree/LeafNode.h"

namespace polyhedralGravity {

    LeafNode::LeafNode(const SplitParam &splitParam)
        : TreeNode(splitParam) {
    }

    void LeafNode::getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) {
        std::for_each(this->splitParam->indexBoundFaces.cbegin(), this->splitParam->indexBoundFaces.cend(), [this, &ray, &origin, &intersections](const size_t faceIndex) {
            if (const std::optional<Array3> intersection = rayIntersectsTriangle(origin, ray, this->splitParam->faces[faceIndex]); intersection.has_value()) {
                intersections.insert(intersection.value());
            }
        });
    }

    std::optional<Array3> LeafNode::rayIntersectsTriangle(const Array3 &rayOrigin, const Array3 &rayVector, const IndexArray3 &triangleVertexIndex) const {
        Array3Triplet edgeVertices{};
        std::transform(triangleVertexIndex.cbegin(), triangleVertexIndex.cend(), edgeVertices.begin(), [this](const size_t vertexIndex) {//transforms a face to its vertices
            return this->splitParam->vertices[vertexIndex];
        });
        return rayIntersectsTriangle(rayOrigin, rayVector, edgeVertices);
    }

    std::optional<Array3> LeafNode::rayIntersectsTriangle(const Array3 &rayOrigin, const Array3 &rayVector, const Array3Triplet &triangleVertices) {
        // Adapted Möller–Trumbore intersection algorithm
        // see https://en.wikipedia.org/wiki/Möller–Trumbore_intersection_algorithm
        using namespace polyhedralGravity;
        using namespace util;
        const Array3 edge1 = triangleVertices[1] - triangleVertices[0];
        const Array3 edge2 = triangleVertices[2] - triangleVertices[0];
        const Array3 h = cross(rayVector, edge2);
        const double a = util::dot(edge1, h);
        if (a > -EPSILON_ZERO_OFFSET && a < EPSILON_ZERO_OFFSET) {
            return {};
        }

        const double f = 1.0 / a;
        const Array3 s = rayOrigin - triangleVertices[0];
        const double u = f * dot(s, h);
        if (u < 0.0 || u > 1.0) {
            return {};
        }

        const Array3 q = cross(s, edge1);
        const double v = f * dot(rayVector, q);
        if (v < 0.0 || u + v > 1.0) {
            return {};
        }

        const double t = f * dot(edge2, q);
        if (t > EPSILON_ZERO_OFFSET) {
            return rayOrigin + rayVector * t;
        } else {
            return {};
        }
}

}