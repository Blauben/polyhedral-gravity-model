#pragma once

#include <memory>

#include "KdDefinitions.h"
#include "TreeNode.h"

namespace polyhedralGravity {

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
        SplitNode(const SplitParam &splitParam, Plane &plane, TriangleIndexLists<2> &triangleIndexLists);
        SplitNode(const SplitNode &other) = delete;
        SplitNode(SplitNode &&other) = delete;
        SplitNode &operator=(const SplitNode &other) = delete;
        SplitNode &operator=(SplitNode &&other) = delete;
        /**
     * Computes the lesser child node if not present already and returns it to the caller.
     * @return the SplitNode that is closer to the origin with respect to the split plane of this node.
     */
        std::shared_ptr<TreeNode> getLesserNode();
        /**
     * Computes the greater child node if not present already and returns it to the caller.
     * @return the SplitNode that is farther away of the origin with respect to the split plane of this node.
     */
        std::shared_ptr<TreeNode> getGreaterNode();
        /**
     * Used to calculated intersections of a ray and the polyhedron's faces contained in this node.
     * @param origin The point where the ray originates from.
     * @param ray Specifies the ray direction.
     * @param intersections The set intersections are added to.
     */
        void getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) override;


    private:
        /**
     * Frees splitParam after both child nodes have been built.
     */
        void maybeFreeParam();
        /**
     * Calculates ray intersections with the bounding box using the concepts of slabs.
     * @param origin The point where the ray originates from.
     * @param ray Specifies the ray direction.
     * @return Returns the t parameter for the entry and exit points of the ray with the box as a pair, with t being from the equation $intersection_point = orig + t * ray$.
     */
        [[nodiscard]] std::pair<double, double> rayBoxIntersection(const Array3 &origin, const Array3 &ray) const;
        /**
     * Intersects a ray with the splitPlane.
     * @param origin The point where the ray originates from.
     * @param ray Specifies the ray direction.
     * @return Returns the t parameter for the intersection point, with t being from the equation $intersection_point = orig + t * ray$.
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