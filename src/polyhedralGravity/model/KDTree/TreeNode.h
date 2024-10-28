#pragma once

#include "polyhedralGravity/model/KDTree/KdDefinitions.h"

namespace polyhedralGravity {

    /**
* Abstract super class for nodes in the {@link KDTree}.
*/
    class TreeNode {
    public:
        virtual ~TreeNode() = default;

    protected:
        /**
    * Protected constructor intended only for child classes. Please use {@link TreeNodeFactory} instead.
*/
        explicit TreeNode(const SplitParam &splitParam, size_t currentRecursionDepth);
        /**
    * Stores parameters required for building child nodes lazily. Gets freed if the Node is an inner node and after both children are built.
    */
        std::unique_ptr<const SplitParam> _splitParam;
        /**
         * the distance to the tree's root from this node. Used to limit the depth and the size of the tree.
         */
        unsigned long _recursionDepth;
    };

}// namespace polyhedralGravity
