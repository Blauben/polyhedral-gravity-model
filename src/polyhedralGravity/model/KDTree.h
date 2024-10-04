#pragma once

#include <optional>
#include "Polyhedron.h"
#include "TreeNode.h"
#include "KdDefinitions.h"

using namespace polyhedralGravity;

class KDTree {
    Direction _currentDirection;
    std::optional<std::unique_ptr<TreeNode>> _rootNode;
    constexpr static const double traverseStepCost{1.0};

public:
    void buildKDTree(const Polyhedron polyhedron);
    bool hasTree() const;

private:
    std::unique_ptr<TreeNode> buildRectangle(const std::vector<Array3>& vertices, const std::vector<IndexArray3>& faces, const std::vector<size_t> indexBoundFaces, const Box boundingBox) const;
    const double surfaceAreaHeuristic(double boundingVolume, double fragmentVolume) const;
   	const Plane findPlane(const std::vector<Array3>& vertices, const std::vector<IndexArray3>& faces, const std::vector<size_t> indexBoundFaces, const Box boundingBox) const;
    const Box getBoundingBox(const std::vector<Array3>& vertices) const;
    const double costForPlane(const Plane plane, const std::vector<Array3>& vertices, const std::vector<IndexArray3>& faces, const std::vector<size_t> indexBoundFaces, const Box boundingBox) const;
    const std::pair<Box,Box> splitBox(const Box& box, const Plane& plane) const;
};
