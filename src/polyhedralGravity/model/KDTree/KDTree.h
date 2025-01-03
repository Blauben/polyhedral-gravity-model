#pragma once

#include "polyhedralGravity/model/KDTree/KdDefinitions.h"
#include "polyhedralGravity/model/KDTree/SplitParam.h"
#include "polyhedralGravity/model/KDTree/TreeNode.h"
#include "polyhedralGravity/model/KDTree/TreeNodeFactory.h"
#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithmFactory.h"

#include <algorithm>
#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <thrust/execution_policy.h>
#include <thrust/for_each.h>
#include <utility>

namespace polyhedralGravity {

    /**
     * A KDTree for a given polyhedron to speed up ray intersections with the polyhedron. It is thread safe.
     */
    class KDTree {
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
         * Set when the root node has been created.
         */
        std::once_flag rootNodeCreated;

    public:
        /**
        * Call to build a KDTree to speed up intersections of rays with a polyhedron's faces.
        * @param vertices The vertex coordinates of the polyhedron
        * @param faces The faces of the polyhedron with a face being a triplet of vertex indices
        * @param algorithm Specifies which algorithm to use for finding optimal split planes.
        * @return the lazily built KDTree.
        */
        KDTree(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, PlaneSelectionAlgorithm::Algorithm algorithm = PlaneSelectionAlgorithm::Algorithm::LOG);

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
       * Parameters for lazily building the root node {@link SplitParam}
       */
        std::unique_ptr<SplitParam> _splitParam;

        void printTree(std::ostream &os) const {
            if (_rootNode != nullptr) {
                os << _rootNode;
            }
        }
    };
}// namespace polyhedralGravity