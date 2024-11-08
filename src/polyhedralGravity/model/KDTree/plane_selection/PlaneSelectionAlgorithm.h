#pragma once

#include "polyhedralGravity/model/KDTree/KdDefinitions.h"

namespace polyhedralGravity {

    class PlaneSelectionAlgorithm {
    public:
        virtual ~PlaneSelectionAlgorithm() = default;
        /**
        * Finds the optimal split plane to split a provided rectangle section optimally.
        * @param splitParam specifies the polyhedron section to be split @link SplitParam.
        * @return Tuple of the optimal plane to split the specified bounding box, its cost as double and a list of triangle sets with respective positions to the found plane. Refer to {@link TriangleIndexLists<2>} for more information.
        */
        virtual std::tuple<Plane, double, std::variant<TriangleIndexLists<2>, PlaneEventLists<2>>> findPlane(const SplitParam &splitParam) = 0;

        enum class Algorithm {
            NOTREE,
            QUADRATIC,
            LOGSQUARED,
            LOG
        };


        /**
        * Constant that describes the cost of traversing the KDTree by one step.
        */
        constexpr static double traverseStepCost{1.0};

        /**
        * Constant that describes the cost of intersecting a ray and a single object.
        */
        constexpr static double triangleIntersectionCost{1.0};

        /**
        * The algorithm used to find optimal split planes.
        */
        static std::unique_ptr<PlaneSelectionAlgorithm> planeSelectionStrategy;

    protected:
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
         * Takes points of a face of a polyhedron and clips them to a box. If all the points lie in the box no changes are made but if points lie outside of the box they are linearly
         * @param box The box to clip the points to.
         * @param points The corner points of the face to be clipped.
         * @return The new corner points of the clipped face.
         */
        static std::vector<Array3> clipToVoxel(const Box &box, const std::array<Array3, 3> &points) {
            using namespace util;
            std::vector input(points.cbegin(), points.cend());
            std::vector<Array3> clipped(points.size());
            //every plane defined by the maxPoint has to flip its normal because the normals have to point inside the bounding box.
            bool flipPlane = false;
            for (const Direction direction : {Direction::X, Direction::Y, Direction::Z}) {
                const auto directionPlanes = {Plane(box.minPoint, direction), Plane(box.maxPoint, direction)};
                for(const auto& plane : directionPlanes) {

                }
            }
                const auto &origin = points[i];
                const auto &dest = points[i + 1 % points.size()];
                const auto ray = dest - origin;
                const auto [t_enter, t_exit] = boundingBox.rayBoxIntersection(origin, ray);
                //box is not hit, no information can be inferred
                if (std::isinf(t_enter) || std::isinf(t_exit)) {
                    continue;
                }
                //intersection point lies in negative ray direction -> origin is inside of box
                //intersection point with box lies in ray direction -> origin outside of box, use intersection point instead
                pushIfAbsent(t_enter <= 0 ? origin : origin + (ray * t_enter));
                pushIfAbsent(t_exit >= 1 ? dest : origin + (ray * t_exit));
            }
            return clipped;
        }
    };

}// namespace polyhedralGravity