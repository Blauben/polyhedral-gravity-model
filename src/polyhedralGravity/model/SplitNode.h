#pragma once

#include <memory>

#include "KdDefinitions.h"
#include "TreeNode.h"

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
         * @plane The plane that splits this node's bounding box into two sub boxes. The child nodes are created based on these boxes.
         * @triangleIndexLists Index sets of the triangles contained in the lesser and greater child nodes. {@link TriangleIndexList}
         */
        SplitNode(const SplitParam &splitParam, Plane &plane, TriangleIndexLists<2> &triangleIndexLists);
        /**
         * Computes the child node decided by the given index (0 for lesser, 1 for greater) if not present already and returns it to the caller.
         * @param index Specifies which node to build. 0 or LESSER for _lesser, 1 or GREATER for _greater.
         * @return the built TreeNode.
        */
        std::shared_ptr<TreeNode> getChildNode(size_t index);
        /**
        * Used to calculated intersections of a ray and the polyhedron's faces contained in this node.
        * @param origin The point where the ray originates from.
        * @param ray Specifies the ray direction.
        * @param intersections The set intersections are added to.
        */
        void getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) override;


    private:
        /**
        * Calculates ray intersections with the bounding box using the concepts of slabs. Refer to https://en.wikipedia.org/wiki/Slab_method.
        * @param origin The point where the ray originates from.
        * @param ray Specifies the ray direction.
        * @return Returns the t parameter for the entry and exit points of the ray with the box as a pair, with t being from the equation $intersection_point = orig + t * ray$.
        */
        [[nodiscard]] std::pair<double, double> rayBoxIntersection(const Array3 &origin, const Array3 &ray) const;
        /**
        * Intersects a ray with the splitPlane.
        * @param origin The point where the ray originates from.
        * @param ray Specifies the ray direction.
        * @return Returns the t parameter for the intersection point, with t being from the equation $intersection_point = orig + t * ray$. Check for NaN in case of parallel plane and ray.
        */
        [[nodiscard]] double rayPlaneIntersection(const Array3 &origin, const Array3 &ray) const;
        /**
         * Gets the children of this node whose bounding boxes are hit by the ray.
         * @param origin The point where the ray originates from.
         * @param ray Specifies the ray direction.
         * @return the child nodes that intersect with the ray
         */
        [[nodiscard]] std::vector<std::shared_ptr<TreeNode>> getChildrenForIntersection(const Array3 &origin, const Array3 &ray);
    };

}// namespace polyhedralGravity