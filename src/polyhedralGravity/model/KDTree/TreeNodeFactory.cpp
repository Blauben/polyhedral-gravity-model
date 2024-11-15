#include "polyhedralGravity/model/KDTree/TreeNodeFactory.h"

namespace polyhedralGravity {

    std::unique_ptr<TreeNode> TreeNodeFactory::createTreeNode(const SplitParam &splitParam, size_t nodeId) {
        //avoid splitting after certain tree depth
        if (recursionDepth(nodeId) >= MAX_RECURSION_DEPTH) {
            return std::make_unique<LeafNode>(splitParam, nodeId);
        }
        const size_t numberOfFaces{countFaces(splitParam.boundFaces)};
        //find optimal plane splitting this node's bounding box
        auto [plane, planeCost, triangleLists] = splitParam.planeSelectionStrategy->findPlane(splitParam);
        const double costWithoutSplit = static_cast<double>(numberOfFaces) * PlaneSelectionAlgorithm::triangleIntersectionCost;

        // Check if the boxes are divided into smaller regions
        bool splitFailsToReduceSize = std::isinf(planeCost) || std::visit([numberOfFaces](auto &typeLists) {
                                          // Count faces in each split box
                                          const size_t facesInMinimalBox = typeLists[0]->size();
                                          const size_t facesInMaximalBox = typeLists[1]->size();

                                          // Ensure that the split meaningfully divides faces
                                          return (numberOfFaces <= facesInMinimalBox + facesInMaximalBox) && (facesInMinimalBox == 0 || facesInMaximalBox == 0);
                                      },
                                                                          triangleLists);

        //if the cost of splitting this node further is greater than just traversing the bound triangles or splitting does not reduce the amount of work in the resulting sub boxes, then don't split and return a LeafNode
        if (planeCost > costWithoutSplit || splitFailsToReduceSize) {
            return std::make_unique<LeafNode>(splitParam, nodeId);
        }
        //if not more costly, perform the split
        return std::make_unique<SplitNode>(splitParam, plane, triangleLists, nodeId);
    }

}// namespace polyhedralGravity