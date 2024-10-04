#pragma once

#include <array>
#include <memory>

#include "KdDefinitions.h"

class TreeNode {
    Plane plane;
    std::unique_ptr<TreeNode> left;
    std::unique_ptr<TreeNode> right;

    public:
        TreeNode(Plane plane);
        TreeNode(const TreeNode& other) = delete;
        TreeNode(TreeNode&& other) = delete;
};