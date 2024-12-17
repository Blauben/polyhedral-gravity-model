#include "polyhedralGravity/model/KDTree/TreeNode.h"

namespace polyhedralGravity {
    TreeNode::TreeNode(const SplitParam &splitParam, const size_t nodeId)
        : nodeId{nodeId}, _splitParam{std::make_unique<SplitParam>(splitParam)} {
    }

}// namespace polyhedralGravity