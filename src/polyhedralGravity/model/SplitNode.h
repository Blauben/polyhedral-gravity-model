#pragma once

#include <memory>

#include "KdDefinitions.h"
#include "TreeNode.h"

class SplitNode : public TreeNode {

    /**
     * The SplitNode that contains the bounding box closer to the origin with respect to the split plane
     */
    std::unique_ptr<TreeNode*> _lesser;
    /**
     * The SplitNode that contains the bounding box further away from the origin with respect to the split plane
     */
    std::unique_ptr<TreeNode*> _greater;
    /**
     * The plane splitting the two TreeNodes contained in this SplitNode
     */
    std::unique_ptr<Plane> _plane;
    /**
     * Contains the triangle lists for the lesser, greater bounding boxes and the triangles which overlap with the split plane. {@link TriangleIndexLists}
     */
    std::unique_ptr<TriangleIndexLists> _triangleIndexLists;

public:
    SplitNode(SplitParam splitParam, Plane& plane, TriangleIndexLists& triangleIndexLists);
    SplitNode(const SplitNode &other) = delete;
    SplitNode(SplitNode &&other) = delete;
    /**
     * Computes the lesser child node if not present already and returns it to the caller.
     * @return the SplitNode that is closer to the origin with respect to the split plane of this node.
     */
    TreeNode* getLesserNode();
    /**
     * Computes the greater child node if not present already and returns it to the caller.
     * @return the SplitNode that is farther away of the origin with respect to the split plane of this node.
     */
    TreeNode* getGreaterNode();
    double intersect() override;

private:
    void maybeFreeParam();
};