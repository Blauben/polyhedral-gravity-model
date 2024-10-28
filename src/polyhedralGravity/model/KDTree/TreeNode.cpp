#include "polyhedralGravity/model/KDTree/TreeNode.h"

namespace polyhedralGravity {
    TreeNode::TreeNode(const SplitParam &splitParam, const size_t currentRecursionDepth)
        : _splitParam{std::make_unique<SplitParam>(splitParam)}, _recursionDepth{currentRecursionDepth} {
    }
}// namespace polyhedralGravity