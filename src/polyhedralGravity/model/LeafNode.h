#pragma once

#include "KdDefinitions.h"
#include "TreeNode.h"

class LeafNode final : public TreeNode {
public:
    explicit LeafNode(const SplitParam &splitParam);
    LeafNode(const LeafNode &other) = delete;
    LeafNode(const LeafNode &&other) = delete;
    LeafNode &operator=(const LeafNode &other) = delete;
    LeafNode &operator=(const LeafNode &&other) = delete;
    /**
     * Used to count the number of intersections of a ray and the polyhedron's faces.
     * @param origin The point where the ray originates from.
     * @param ray Specifies the ray direction.
     * @return The number of intersections with the polyhedron.
     */
    unsigned long countIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray) override;
    void getFaceIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray, std::vector<size_t> &intersectedFaceIndices) override;


private:
    std::unique_ptr<polyhedralGravity::Array3> rayIntersectsTriangle(const polyhedralGravity::Array3 &rayOrigin, const polyhedralGravity::Array3 &rayVector, const polyhedralGravity::IndexArray3 &triangleVertexIndex) const;
    static std::unique_ptr<polyhedralGravity::Array3> rayIntersectsTriangle(const polyhedralGravity::Array3 &rayOrigin, const polyhedralGravity::Array3 &rayVector, const polyhedralGravity::Array3Triplet &triangleVertices);
};