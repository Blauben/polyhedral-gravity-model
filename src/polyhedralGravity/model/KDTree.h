#pragma once

#include <optional>
#include "Polyhedron.h"
#include "TreeNode.h"
#include "KdDefinitions.h"

using namespace polyhedralGravity;

class KDTree {
    std::optional<std::unique_ptr<TreeNode>> _rootNode;
    constexpr static const double traverseStepCost{1.0};
    constexpr static const double triangleIntersectionCost{1.0};

public:
    void buildKDTree(const Polyhedron polyhedron);
    bool hasTree() const;

private:
    std::unique_ptr<TreeNode> buildRectangle(const SplitParam& param) const;
    const double surfaceAreaHeuristic(const double boundingVolume, const double fragmentVolume) const;
   	const Plane findPlane(const SplitParam& param) const;
    const Box getBoundingBox(const std::vector<Array3>& vertices) const;
    const double costForPlane(const SplitParam& param, const Plane& plane) const;
    const std::pair<Box,Box> splitBox(const Box& box, const Plane& plane) const;
    const double volumeOfBox(const Box& box) const;
	const std::array<std::vector<size_t>, 3> containedTriangles(const SplitParam& param, const Plane& split) const;
};
