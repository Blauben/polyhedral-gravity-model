#pragma once

#include "KdDefinitions.h"
#include "TreeNode.h"


class KDTree {
public:
    /**
     * Call to build a KDTree to speed up intersections of rays with a polyhedron's faces.
     * @param vertices The vertex coordinates of the polyhedron
     * @param faces The faces of the polyhedron with a face being a triplet of vertex indices
     * @return the lazily built KDTree.
     */
    KDTree(const std::vector<polyhedralGravity::Array3>& vertices, const std::vector<polyhedralGravity::IndexArray3>& faces);

    KDTree(KDTree&& other) noexcept ;

    KDTree(const KDTree&) = delete;

    KDTree& operator=(KDTree& other) = delete;

    KDTree& operator=(KDTree&& other) noexcept ;

    bool operator==(const KDTree & other) const;

    /**
     * Finds the optimal split plane to split a provided rectangle section optimally.
     * @param param specifies the polyhedron section to be split @link SplitParam.
     * @return Tuple of the optimal plane to split the specified bounding box, its cost as double and a list of triangle sets with respective positions to the found plane. Refer to {@link TriangleIndexLists<2>} for more information.
     */
    static std::tuple<Plane, double, TriangleIndexLists<2>> findPlane(const SplitParam &param);// O(N^2) implementation

    /**
     * Splits a box into two new boxes.
     * @param box the box to be split.
     * @param plane the plane by which to split the original box.
     * @return a pair of boxes that result by splitting the provided box.
     */
    static std::pair<Box, Box> splitBox(const Box &box, const Plane &plane);

    /**
     * Constant that describes the cost of traversing the KDTree by one step.
     */
    constexpr static const double traverseStepCost{1.0};
    /**
     * Constant that describes the cost of intersecting a ray and a single object.
     */
    constexpr static const double triangleIntersectionCost{1.0};

    /**
     * Creates the root tree node if not initialized and returns it.
     * @return the root tree Node.
     */
    TreeNode &getRootNode();

    /**
     * Used to count the number of intersections of a ray and the polyhedron's faces.
     * @param origin The point where the ray originates from.
     * @param ray Specifies the ray direction.
     * @return The number of intersections with the polyhedron.
     */
    unsigned long countIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray);

    void getFaceIntersections(const polyhedralGravity::Array3 &origin, const polyhedralGravity::Array3 &ray, std::vector<size_t> &intersectedFaceIndices);


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
     * Finds the minimal bounding box for a set of vertices.
     * @param vertices the set of vertex coordinates for which to find the box
     * @return the bounding box @link Box
     */
    static Box getBoundingBox(const std::vector<polyhedralGravity::Array3> &vertices);
    /**
     * Evaluates the cost function should the specified bounding box and it's faces be divided by the specified plane. Used to evaluate possible split planes.
     * @param param specifies the polyhedron section to be split @link SplitParam.
     * @param plane the candidate split plane to be evaluated.
     * @return the cost for performing intersection operations on the finalized tree later, should the KDTree be built using the specified split plane and the triangle sets resulting through division by the plane.
     */
    static std::pair<const double, TriangleIndexLists<2>> costForPlane(const SplitParam &param, const Plane &plane);
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
     * @return Two triangle lists contained in an array. Those being the set of triangles with non-zero area in the bounding box closer to the origin with respect to the split plane,
     * the set of triangles with non-zero area in the bounding box further away from the origin with respect to the split plane.
     * The set of triangles that lies on the plane is already added to the above set which achieves maximum performance.
     */
    static TriangleIndexLists<3> containedTriangles(const SplitParam &param, const Plane &split);
};
