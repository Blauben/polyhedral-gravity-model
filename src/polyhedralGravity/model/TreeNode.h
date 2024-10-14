#pragma once

#include "KdDefinitions.h"

namespace polyhedralGravity {

    /**
* Abstract super class for nodes in the {@link KDTree}.
*/
    class TreeNode {
    public:
        virtual ~TreeNode() = default;
        TreeNode(const TreeNode &) = delete;
        TreeNode(TreeNode &&) = delete;
        TreeNode &operator=(const TreeNode &) = delete;
        TreeNode &operator=(TreeNode &&) = delete;
        /**
     * Used to calculated intersections of a ray and the polyhedron's faces contained in this node.
     * @param origin The point where the ray originates from.
     * @param ray Specifies the ray direction.
     * @param intersections The set intersections are added to.
     */
        virtual void getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) = 0;

    protected:
        /**
    * Protected constructor for child classes. Please use factory method instead.
*/
        explicit TreeNode(const SplitParam &splitParam);
        /**
    * Stores parameters required for building child nodes lazily. Gets freed after children are built if the Node is an inner node.
    */
        std::unique_ptr<const SplitParam> splitParam;
    };

}// namespace polyhedralGravity
