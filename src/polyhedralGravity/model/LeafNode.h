#pragma once

#include "TreeNode.h"
#include <vector>

class LeafNode : TreeNode {
    std::vector<size_t> indexBoundFaces;
    public:
        double intersect() override;
};