#pragma once

#include "polyhedralGravity/model/KDTree/KDTree.h"
#include "polyhedralGravity/model/KDTree/KdDefinitions.h"
#include "polyhedralGravity/model/KDTree/TreeNode.h"
#include "polyhedralGravity/model/KDTree/TreeNodeFactory.h"

#include <algorithm>
#include <array>
#include <memory>
#include <unordered_set>
#include <utility>
#include <thrust/iterator/transform_iterator.h>

namespace polyhedralGravity {

    /**
     * A KDTree for a given polyhedron to speed up ray intersections with the polyhedron
     */
    class KDTree {
    public:
        /**
        * Call to build a KDTree to speed up intersections of rays with a polyhedron's faces.
        * @param vertices The vertex coordinates of the polyhedron
        * @param faces The faces of the polyhedron with a face being a triplet of vertex indices
        * @return the lazily built KDTree.
        */
        KDTree(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces);

        // O(N^2) implementation
        /**
        * Finds the optimal split plane to split a provided rectangle section optimally.
        * @param param specifies the polyhedron section to be split @link SplitParam.
        * @return Tuple of the optimal plane to split the specified bounding box, its cost as double and a list of triangle sets with respective positions to the found plane. Refer to {@link TriangleIndexLists<2>} for more information.
        */
        static std::tuple<Plane, double, TriangleIndexLists<2>> findPlane(const SplitParam &param);

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
        constexpr static double traverseStepCost{1.0};
        /**
        * Constant that describes the cost of intersecting a ray and a single object.
        */
        constexpr static double triangleIntersectionCost{1.0};

        /**
        * Creates the root tree node if not initialized and returns it.
        * @return the root tree Node.
        */
        TreeNode &getRootNode();

        /**
        * Used to calculate intersections of a ray and the polyhedron's faces contained in this node.
        * @param origin The point where the ray originates from.
        * @param ray Specifies the ray direction.
        * @param intersections The set found intersection points are added to.
        */
        void getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections);

        /**
         * Calculates the number of intersections of a ray with the polyhedron.
         * @param origin The origin point of the ray.
         * @param ray The ray direction vector.
         * @return the number of intersections.
         */
        size_t countIntersections(const Array3 &origin, const Array3 &ray);

        /**
        * Finds the minimal bounding box for a set of vertices.
        * @param vertices the set of vertex coordinates for which to find the box
        * @return the bounding box {@link Box}
        */
        template<typename Container>
        static Box getBoundingBox(const Container &vertices);

        /**
        * An iterator transforming face indices to vertices and returning both.
        * This function returns a pair of transform iterators (first = begin(), second = end()).
        * @param begin begin iterator of the face indice vector to transform.
        * @param end end iterator of the face indice vector to transform.
        * @param vertices the vector of vertices to look up the indices obtained from the faces vector.
        * @param faces the faces vector to lookup face indices.
        * @return pair of transform iterators.
        */
        [[nodiscard]] static inline auto transformIterator(const std::vector<unsigned long>::const_iterator begin, const std::vector<unsigned long>::const_iterator end, const std::vector<Array3>& vertices, const std::vector<IndexArray3>& faces) {
            //The offset must be captured by value to ensure its lifetime!
            const auto lambdaApplication = [&vertices, &faces](unsigned long faceIndex) {
                const auto& face = faces[faceIndex];
                Array3Triplet vertexTriplet =  {
                    vertices[face[0]],
                    vertices[face[1]],
                    vertices[face[2]]
                } ;
                return std::make_pair(faceIndex, vertexTriplet);
            };


            auto first = thrust::make_transform_iterator(begin, lambdaApplication);
            auto last = thrust::make_transform_iterator(end, lambdaApplication);
            return std::make_pair(first, last);
        }


    private:
        /**
        * The entry node of the KDTree. Only access using getter.
        */
        std::unique_ptr<TreeNode> _rootNode;

        /**
         * The polyhedron's vertices.
         */
        const std::vector<Array3> _vertices;
        /**
         * The polyhedron's faces: A face is a triplet of vertex indices.
         */
        const std::vector<IndexArray3> _faces;


        /**
        * Parameters for lazily building the root node {@link SplitParam}
        */
        std::unique_ptr<SplitParam> _splitParam;

        /**
        * Evaluates the cost function should the specified bounding box and it's faces be divided by the specified plane. Used to evaluate possible split planes.
        * @param splitParam specifies the polyhedron section to be split {@link SplitParam}.
        * @param plane the candidate split plane to be evaluated.
        * @return the cost for performing intersection operations on the finalized tree later, should the KDTree be built using the specified split plane and the triangle sets resulting through division by the plane.
        */
        static std::pair<const double, TriangleIndexLists<2>> costForPlane(const SplitParam &splitParam, const Plane &plane);
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
        * @return Three triangle lists contained in an array. Those being the set of triangles with non-zero area in the bounding box closer to the origin with respect to the split plane,
        * the set of triangles with non-zero area in the bounding box further away from the origin with respect to the split plane.
        * The set of triangles that lies on the plane.
        */
        static TriangleIndexLists<3> containedTriangles(const SplitParam &param, const Plane &split);
    };

}// namespace polyhedralGravity