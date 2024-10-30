#pragma once

#include "polyhedralGravity/model/KDTree/KdDefinitions.h"
#include "polyhedralGravity/model/KDTree/TreeNode.h"
#include "polyhedralGravity/model/KDTree/TreeNodeFactory.h"
#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithm.h"

#include <algorithm>
#include <array>
#include <deque>
#include <memory>
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
        * @param algorithm Specifies which algorithm to use for finding optimal split planes.
        * @return the lazily built KDTree.
        */
        KDTree(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, PlaneSelectionAlgorithm::Algorithm algorithm);

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

    };

}// namespace polyhedralGravity