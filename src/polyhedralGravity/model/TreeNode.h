#pragma once

#include <array>
#include <memory>

using Plane = std::array<double,2>; //TODO find suitable format

class TreeNode {
    Plane plane;
    std::unique_ptr<TreeNode> left;
    std::unique_ptr<TreeNode> right;

    public:
        TreeNode(Plane plane);
        TreeNode(const TreeNode& other) = delete;
        TreeNode(TreeNode&& other) = delete;
};