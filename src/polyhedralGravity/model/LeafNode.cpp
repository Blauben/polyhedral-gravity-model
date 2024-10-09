#include "LeafNode.h"

LeafNode::LeafNode(SplitParam splitParam) : TreeNode(splitParam) {}

double LeafNode::intersect() {
    return 1.0;
}