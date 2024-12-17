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
        * @param nodeId The unique id to be assigned to the newly created node. Follows the convention that the left child gets the id 2 * <current_id> + 1 and
        * the right child 2 * <currrent_id> + 2.
        * @return A unique pointer to the new TreeNode.
         */
        static std::unique_ptr<TreeNode> createTreeNode(const SplitParam &splitParam, size_t nodeId);
    };

}// namespace polyhedralGravity