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

        //TODO: rework, continue here
        /**
         * Clip points or coordinates to a box only adjusting the values in a specific dimension.
         * @param box The box to which to clip to.
         * @param direction The dimension in which to clip.
         * @param values The points or coordinates to clip.
         * @return An array of clipped values.
         */
        template<typename... Values>
        static std::array<double, sizeof...(Values)> clipToVoxel(const Box &box, const Direction direction, Values... values) {
            return {clipToVoxel(box, direction, values)...};
        }

        /**
         * Clip coordinate to a box in a specific direction.
         * @param box The box to clip to.
         * @param direction The direction that should be clipped in.
         * @param coordinate The coordinate of dimension direction to clip to the box.
         * @return The clipped coordinates.
         */
        template<typename Coordinate>
        static Coordinate clipToVoxel(const Box &box, const Direction direction, Coordinate coordinate) {
            if (coordinate < box.minPoint[static_cast<int>(direction)]) {
                return box.minPoint[static_cast<int>(direction)];
            }
            if (coordinate > box.maxPoint[static_cast<int>(direction)]) {
                return box.maxPoint[static_cast<int>(direction)];
            }
            return coordinate;
        }

        /**
         * Clip points to a box in a specific direction.
         * @param box The box to clip to.
         * @param direction The direction that should be clipped in.
         * @param point A container of double coordinates.
         * @return The clipped coordinates.
         */
        template<template<typename, size_t> typename Point, typename Coordinate, size_t dimension>
        static Coordinate clipToVoxel(const Box &box, const Direction direction, Point<Coordinate, dimension> point) {
            return {clipToVoxel(box, direction, point[static_cast<size_t>(direction)])};
        }
    };

}// namespace polyhedralGravity