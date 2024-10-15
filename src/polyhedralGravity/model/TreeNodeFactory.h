#pragma once

#include "TreeNode.h"
#include <memory>

namespace polyhedralGravity {

    /**
     * Factory class for building TreeNodes. {@link TreeNode}
     */
    class TreeNodeFactory {
    public:
        /**
     * Builds a new TreeNode for a KDTree. {@link KDTree}
     * @param splitParam Parameters for intersection testing and child node creation. {@link SplitParam}
     * @return A unique pointer to the new TreeNode.
     */
        static std::unique_ptr<TreeNode> treeNodeFactory(const SplitParam &splitParam);
    };

}// namespace polyhedralGravity