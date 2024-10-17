#include "TreeNode.h"

namespace polyhedralGravity {
    TreeNode::TreeNode(const SplitParam &splitParam)
        : splitParam(std::make_unique<SplitParam>(splitParam)) {
    }
}// namespace polyhedralGravity