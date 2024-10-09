#pragma once

#include <utility>

#include "KdDefinitions.h"
#include "Polyhedron.h"
#include "TreeNode.h"

class KDTree {
    /**
     * Constant that describes the cost of traversing the KDTree by one step.
     */
    constexpr static const double traverseStepCost{1.0};
    /**
     * Constant that describes the cost of intersecting a ray and a single object.
     */
    constexpr static const double triangleIntersectionCost{1.0};

public:
    /**
     * Call to build a KDTree to speed up intersections of rays with a polyhedron's faces.
     * @param polyhedron The polyhedron for which to build the KDTree.
     * @return the lazily built KDTree.
     */
    explicit KDTree(const polyhedralGravity::Polyhedron &polyhedron);

    /**
     * Finds the optimal split plane to split a provided rectangle section optimally.
     * @param param specifies the polyhedron section to be split @link SplitParam.
     * @return Pair of the optimal plane to split the specified bounding box and a list of triangle sets with respective positions to the found plane. Refer to {@link TriangleIndexLists} for more information.
     */
    static std::pair<Plane, TriangleIndexLists> findPlane(const SplitParam &param);// O(N^2) implementation

    /**
     * Splits a box into two new boxes.
     * @param box the box to be split.
     * @param plane the plane by which to split the original box.
     * @return a pair of boxes that result by splitting the provided box.
     */
    static std::pair<Box, Box> splitBox(const Box &box, const Plane &plane);


private:
    /**
     * The entry node of the KDTree. Only access using getter.
     */
    std::unique_ptr<TreeNode> rootNode;

    /**
     * Parameters for lazily building the root node @link SplitParam
     */
    std::unique_ptr<SplitParam> param;

    /**
     * Creates the root tree node if not initialized and returns it.
     * @return the root tree Node.
     */
    TreeNode & getRootNode();

    /**
     * Finds the minimal bounding box for a set of vertices.
     * @param vertices the set of vertex coordinates for which to find the box
     * @return the bounding box @link Box
     */
    static Box getBoundingBox(const std::vector<polyhedralGravity::Array3> &vertices);
    /**
     * Evaluates the cost function should the specified bounding box and it's faces be divided by the specified plane. Used to evaluate possible split planes.
     * @param param specifies the polyhedron section to be split @link SplitParam.
     * @param plane the candidate split plane to be evaluated.
     * @return the cost for performing intersection operations on the finalized tree later, should the KDTree be built using the specified split plane.
     */
    static std::pair<const double, TriangleIndexLists> costForPlane(const SplitParam &param, const Plane &plane);
    /**
     * Calculates the surface area of a box.
     * @param box specifies the box to be used.
     * @return the surface area
     */
    static double surfaceAreaOfBox(const Box &box);
    /**
     * Splits a section of a polyhedron into two bounding boxes and calculates the triangle face sets contained in the new bounding boxes.
     * @param param specifies the polyhedron section to be split.
     * @param split the plane by which to split the polyhedron section.
     * @return Three triangle sets contained in an array. Those being the set of triangles with non-zero area in the bounding box closer to the origin with respect to the split plane,
     * the set of triangles with non-zero area in the bounding box further away from the origin with respect to the split plane
     * and the set of triangles that overlap with the split plane itself.
     */
    static TriangleIndexLists containedTriangles(const SplitParam &param, const Plane &split);
};
