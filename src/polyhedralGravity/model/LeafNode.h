#pragma once

#include "KdDefinitions.h"
#include "TreeNode.h"

#include <optional>

namespace polyhedralGravity {

    class LeafNode final : public TreeNode {
    public:
        explicit LeafNode(const SplitParam &splitParam);
        LeafNode(const LeafNode &other) = delete;
        LeafNode(const LeafNode &&other) = delete;
        LeafNode &operator=(const LeafNode &other) = delete;
        LeafNode &operator=(const LeafNode &&other) = delete;
        /**
    * Used to calculated intersections of a ray and the polyhedron's faces contained in this node.
    * @param origin The point where the ray originates from.
    * @param ray Specifies the ray direction.
    * @param intersections The set intersections are added to.
    */
        void getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) override;


    private:
        [[nodiscard]] std::optional<Array3> rayIntersectsTriangle(const Array3 &rayOrigin, const Array3 &rayVector, const IndexArray3 &triangleVertexIndex) const;
        static std::optional<Array3> rayIntersectsTriangle(const Array3 &rayOrigin, const Array3 &rayVector, const Array3Triplet &triangleVertices);
    };

}// namespace polyhedralGravity