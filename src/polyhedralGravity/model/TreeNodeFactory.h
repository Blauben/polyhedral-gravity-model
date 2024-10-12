#pragma once

#include <memory>
#include "TreeNode.h"

namespace polyhedralGravity {

class TreeNodeFactory {
public:
    static std::unique_ptr<TreeNode> treeNodeFactory(const SplitParam &splitParam);
};

}