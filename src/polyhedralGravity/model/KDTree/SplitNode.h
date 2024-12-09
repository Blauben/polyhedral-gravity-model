#pragma once

#include <memory>

#include "polyhedralGravity/model/KDTree/KDTree.h"
#include "polyhedralGravity/model/KDTree/KdDefinitions.h"
#include "polyhedralGravity/model/KDTree/TreeNode.h"
#include "polyhedralGravity/model/KDTree/TreeNodeFactory.h"

#include <algorithm>

#define LESSER 0
#define GREATER 1

namespace polyhedralGravity {

    /**
     * A TreeNode contained in a KDTree that splits the spatial hierarchy into two new sub boxes. Intersection tests are delegated to the child nodes.
     */
    class SplitNode final : public TreeNode {

        /**
        * The SplitNode that contains the bounding box closer to the origin with respect to the split plane
        */
        std::shared_ptr<TreeNode> _lesser;
        /**
        * The SplitNode that contains the bounding box further away from the origin with respect to the split plane
        */
        std::shared_ptr<TreeNode> _greater;
        /**
        * The plane splitting the two TreeNodes contained in this SplitNode
        */
        Plane _plane;
        /**
         * the bounding box for all triangles contained in this node.
         */
        Box _boundingBox;
        /**
         * Contains the triangle lists for the lesser and greater bounding boxes. {@link TriangleIndexLists}
        */
        TriangleIndexLists<2> _triangleIndexLists;

    public:
        /**
         * Takes parameters from the parent node and stores them for lazy child node creation.
         * @param splitParam Parameters produced during the split that resulted in the creation of this node.
         * @param plane The plane that splits this node's bounding box into two sub boxes. The child nodes are created based on these boxes.
         * @param triangleIndexLists Index sets of the triangles contained in the lesser and greater child nodes. {@link TriangleIndexList}
         * @param currentRecursionDepth the tree depth of the current node. Used to limit the size of the tree.
         */
        SplitNode(const SplitParam &splitParam, const Plane &plane, TriangleIndexLists<2> &triangleIndexLists, size_t currentRecursionDepth);
        /**
         * Computes the child node decided by the given index (0 for lesser, 1 for greater) if not present already and returns it to the caller.
         * @param index Specifies which node to build. 0 or LESSER for _lesser, 1 or GREATER for _greater.
         * @return the built TreeNode.
        */
        std::shared_ptr<TreeNode> getChildNode(size_t index);
        /**
         * Gets the children of this node whose bounding boxes are hit by the ray.
         * @param origin The point where the ray originates from.
         * @param ray Specifies the ray direction.
         * @return the child nodes that intersect with the ray
         */
        [[nodiscard]] std::vector<std::shared_ptr<TreeNode>> getChildrenForIntersection(const Array3 &origin, const Array3 &ray);


    private:
        /**
        * Intersects a ray with the splitPlane.
        * @param origin The point where the ray originates from.
        * @param ray Specifies the ray direction.
        * @return Returns the t parameter for the intersection point, with t being from the equation $intersection_point = orig + t * ray$. Check for NaN in case of parallel plane and ray.
        */
        [[nodiscard]] double rayPlaneIntersection(const Array3 &origin, const Array3 &ray) const;
    };

}// namespace polyhedralGravity