#pragma once

#include "polyhedralGravity/model/KDTree/LeafNode.h"
#include "polyhedralGravity/model/KDTree/SplitNode.h"
#include "polyhedralGravity/model/KDTree/TreeNode.h"
#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithm.h"


#include <memory>

constexpr uint8_t MAX_RECURSION_DEPTH{64};

namespace polyhedralGravity {

    /**
     * Factory class for building TreeNodes. {@link TreeNode}
     */
    class TreeNodeFactory {
    public:
        /**
        * Builds a new TreeNode for a KDTree. {@link KDTree}
        * @param splitParam Parameters for intersection testing and child node creation. {@link SplitParam}
        * @param currentRecursionDepth The tree depth of the new node requested to be built.
        * @return A unique pointer to the new TreeNode.
         */
        static std::unique_ptr<TreeNode> createTreeNode(const SplitParam &splitParam, size_t currentRecursionDepth);
    };

}// namespace polyhedralGravity