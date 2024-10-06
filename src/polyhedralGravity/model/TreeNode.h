#pragma once

#include <array>
#include <memory>

#include "KdDefinitions.h"

class TreeNode {

public:
    explicit TreeNode(Plane plane);
    TreeNode(const TreeNode &other) = delete;
    TreeNode(TreeNode &&other) = delete;
    Plane plane;
    std::unique_ptr<TreeNode> left;
    std::unique_ptr<TreeNode> right;
};