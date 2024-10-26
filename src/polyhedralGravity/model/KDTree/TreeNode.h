#pragma once

#include "polyhedralGravity/model/KDTree/KdDefinitions.h"

namespace polyhedralGravity {

    /**
* Abstract super class for nodes in the {@link KDTree}.
*/
    class TreeNode {
    public:
        virtual ~TreeNode() = default;
        /**
     * Used to calculated intersections of a ray and the polyhedron's faces contained in this node.
     * @param origin The point where the ray originates from.
     * @param ray Specifies the ray direction.
     * @param intersections The set intersections are added to.
     */
        virtual void getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) = 0;
        unsigned long recursionDepth{0};

    protected:
        /**
    * Protected constructor intended only for child classes. Please use {@link TreeNodeFactory} instead.
*/
        explicit TreeNode(const SplitParam &splitParam);
        /**
    * Stores parameters required for building child nodes lazily. Gets freed if the Node is an inner node and after both children are built.
    */
        std::unique_ptr<const SplitParam> _splitParam;
    };

}// namespace polyhedralGravity
