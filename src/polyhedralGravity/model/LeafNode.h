#pragma once

#include "TreeNode.h"
#include "KdDefinitions.h"

class LeafNode : public TreeNode {
    public:
        explicit LeafNode(const SplitParam& splitParam);
        unsigned long countIntersections(const polyhedralGravity::Array3& origin, const polyhedralGravity::Array3& ray) override;

private:
    static std::unique_ptr<polyhedralGravity::Array3> rayIntersectsTriangle(const polyhedralGravity::Array3 &rayOrigin, const polyhedralGravity::Array3 &rayVector, const polyhedralGravity::Array3Triplet &triangle);
};