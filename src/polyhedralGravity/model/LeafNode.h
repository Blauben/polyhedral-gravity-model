#pragma once

#include "KdDefinitions.h"
#include "TreeNode.h"

class LeafNode final : public TreeNode {
public:
    explicit LeafNode(const SplitParam& splitParam);
    LeafNode(const LeafNode& other) = delete;
    LeafNode(const LeafNode&& other) = delete;
    LeafNode& operator=(const LeafNode& other) = delete;
    LeafNode& operator=(const LeafNode&& other) = delete;
    unsigned long countIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray) override;

private:
    static std::unique_ptr<polyhedralGravity::Array3> rayIntersectsTriangle(const polyhedralGravity::Array3 &rayOrigin, const polyhedralGravity::Array3 &rayVector, const polyhedralGravity::Array3Triplet &triangle);
};