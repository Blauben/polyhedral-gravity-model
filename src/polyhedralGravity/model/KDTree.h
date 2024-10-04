#pragma once

#include <optional>
#include "Polyhedron.h"
#include "TreeNode.h"

using namespace polyhedralGravity;

class KDTree {

    std::optional<TreeNode> _rootNode;

public:
    void buildKDTree(const Polyhedron polyhedron);
    bool hasTree() const;

private:
    std::unique_ptr<TreeNode> buildRectangle();
};
