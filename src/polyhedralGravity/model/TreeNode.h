#pragma once

#include "KdDefinitions.h"

/**
* Abstract super class for nodes in the {@link KDTree}.
*/
class TreeNode {
public:
    virtual ~TreeNode() = default;
    TreeNode(TreeNode&) = delete;
    TreeNode(TreeNode&&) = delete;
    /**
    * Used to compute intersections with the KDTree nodes and propagate to the correct childNodes if necessary.
    * @param TODO
    * @return TODO
    */
    virtual double intersect() = 0;//TODO: find correct return and param types
    static std::unique_ptr<TreeNode*> treeNodeFactory(SplitParam splitParam);

protected:
    /**
    * Protected constructor for child classes. Please use factory method instead.
*/
    explicit TreeNode(SplitParam splitParam);
    /**
    * Stores parameters required for building child nodes lazily. Gets freed after children are built.
    */
    std::unique_ptr<SplitParam> splitParam;
};