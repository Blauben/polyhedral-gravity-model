#pragma once

#include "polyhedralGravity/model/KDTree/KDTree.h"
#include "polyhedralGravity/model/KDTree/KdDefinitions.h"
#include "polyhedralGravity/model/KDTree/TreeNode.h"
#include "polyhedralGravity/model/KDTree/TreeNodeFactory.h"

#include <algorithm>
#include <array>
#include <memory>
#include <queue>
#include <thrust/iterator/transform_iterator.h>
#include <unordered_set>
#include <utility>

constexpr uint8_t MIN{0};
constexpr uint8_t MAX{1};

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

        // O(N*log^2(N)) implementation
        /**
        * Finds the optimal split plane to split a provided rectangle section optimally.
        * @param splitParam specifies the polyhedron section to be split @link SplitParam.
        * @return Tuple of the optimal plane to split the specified bounding box, its cost as double and a list of triangle sets with respective positions to the found plane. Refer to {@link TriangleIndexLists<2>} for more information.
        */
        static std::tuple<Plane, double, TriangleIndexLists<2>> findPlane(const SplitParam &splitParam);

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
        std::shared_ptr<TreeNode> getRootNode();

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
        [[nodiscard]] static inline auto transformIterator(const std::vector<unsigned long>::const_iterator begin, const std::vector<unsigned long>::const_iterator end, const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces) {
            //The offset must be captured by value to ensure its lifetime!
            const auto lambdaApplication = [&vertices, &faces](unsigned long faceIndex) {
                const auto &face = faces[faceIndex];
                Array3Triplet vertexTriplet = {
                        vertices[face[0]],
                        vertices[face[1]],
                        vertices[face[2]]};
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
        std::shared_ptr<TreeNode> _rootNode;

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
        * @param boundingBox the bounding box encompassing the scene to be split.
        * @param plane the candidate split plane to be evaluated.
        * @param trianglesMin the number of triangles overlapping with the min side of the bounding box.
        * @param trianglesMax the number of triangles overlapping with the max side of the bounding box.
        * @param trianglesPlanar the number of triangles lying in the plane.
        * @return A pair of: 1. the cost for performing intersection operations on the finalized tree later, should the KDTree be built using the specified split plane and the triangle sets resulting through division by the plane.
        * 2. true if the planar triangles should be added to the min side of the bounding box.
        */
        static std::pair<const double, bool> costForPlane(const Box &boundingBox, const Plane &plane, size_t trianglesMin, size_t trianglesMax, size_t trianglesPlanar);

        /**
         * Generates the vector of PlaneEvents comprising all the possible candidate planes. {@link PlaneEvent}
         * @param splitParam Contains the parameters of the scene to find candidate planes for. {@link SplitParam}
         * @return The vector of PlaneEvents
         */
        static std::vector<PlaneEvent> generatePlaneEvents(const SplitParam &splitParam);

        /**
         * When an optimal plane has been found extract the index lists of faces for further subdivision through child nodes.
         * @param planeEvents The events that were generated during {@link findPlane}.
         * @param plane The plane to split the faces by.
         * @param minSide Whether to include planar faces to the bounding box closer to the origin.
         * @return The triangleIndexlists for the bounding boxes closer and further away from the origin.
         */
        static TriangleIndexLists<2> generateTriangleSubsets(const std::vector<PlaneEvent> &planeEvents, const Plane &plane, bool minSide);

        /**
         * Clip points to a box in a specific direction.
         * @param box The box to clip to.
         * @param direction The direction that should be clipped in.
         * @param points
         * @return The clipped coordinates.
         */
        template<typename... Points>
        static std::array<double, sizeof...(Points)> clipToVoxel(const Box &box, Direction direction, Points... points);

        /**
        * Calculates the surface area of a box.
        * @param box specifies the box to be used.
        * @return the surface area
        */
        static double surfaceAreaOfBox(const Box &box);
    };

}// namespace polyhedralGravity