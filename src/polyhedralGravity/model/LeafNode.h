#pragma once

#include "TreeNode.h"

class LeafNode final : public TreeNode {
    public:
        explicit LeafNode(SplitParam splitParam);
        double intersect() override;
};