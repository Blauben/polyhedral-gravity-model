#include "TreeNode.h"

TreeNode::TreeNode(SplitParam splitParam) : splitParam(std::make_unique<SplitParam>(splitParam)) {}