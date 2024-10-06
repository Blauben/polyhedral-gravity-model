#pragma once

#include "KdDefinitions.h"
#include "Polyhedron.h"
#include "TreeNode.h"
#include <optional>

using namespace polyhedralGravity;

using TriangleIndexLists = std::array<std::unique_ptr<std::vector<size_t>>, 3>;

class KDTree {
    constexpr static const double traverseStepCost{1.0};
    constexpr static const double triangleIntersectionCost{1.0};

public:
    std::optional<std::unique_ptr<TreeNode>> rootNode;
    void buildKDTree(const Polyhedron &polyhedron);
    [[nodiscard]] bool hasTree() const;

private:
    [[nodiscard]] std::unique_ptr<TreeNode> buildRectangle(const SplitParam &param) const;
    [[nodiscard]] std::pair<Plane, TriangleIndexLists> findPlane(const SplitParam &param) const;// O(N^2) implementation
    static Box getBoundingBox(const std::vector<Array3> &vertices);
    static std::pair<const double, TriangleIndexLists> costForPlane(const SplitParam &param, const Plane &plane);
    static std::pair<Box, Box> splitBox(const Box &box, const Plane &plane);
    static double surfaceAreaOfBox(const Box &box);
    static TriangleIndexLists containedTriangles(const SplitParam &param, const Plane &split);
};
