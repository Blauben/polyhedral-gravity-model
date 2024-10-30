#pragma once

#include "polyhedralGravity/model/KDTree/KDTree.h"
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
        virtual std::tuple<Plane, double, TriangleIndexLists<2>> findPlane(const SplitParam &splitParam) = 0;

        enum class Algorithm {
            NOTREE,
            QUADRATIC,
            LOGSQUARED,
            LOG
        };

        /**
         * Factory method to return the plane finding selection algorithm specified by the enum parameter.
         * @param algorithm Specifies which algorithm to return.
         * @return The algorithm which executes the requested strategy.
         */
        static std::unique_ptr<PlaneSelectionAlgorithm> create(Algorithm algorithm);

        /**
        * Constant that describes the cost of traversing the KDTree by one step.
        */
        constexpr static double traverseStepCost{1.0};

        /**
        * Constant that describes the cost of intersecting a ray and a single object.
        */
        constexpr static double triangleIntersectionCost{1.0};

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
         * Clip points to a box in a specific direction.
         * @param box The box to clip to.
         * @param direction The direction that should be clipped in.
         * @param points
         * @return The clipped coordinates.
         */
        template<typename... Points>
        static std::array<double, sizeof...(Points)> clipToVoxel(const Box &box, Direction direction, Points... points);
    };

    /**
    * Used by {@link PlaneEvent} to position the face that generated the event relative to the generated plane.
    */
    enum class PlaneEventType {
        ending = 0,
        planar = 1,
        starting = 2,
    };

    /**
     * Generated when traversing the vector of faces and building their candidate planes.
     */
    struct PlaneEvent {
        PlaneEventType type;
        /**
         * The candidate plane suggested by the face included in this struct.
         */
        Plane plane;
        /**
         * The index of the face that generated this candidate plane.
         */
        unsigned faceIndex;

        PlaneEvent(const PlaneEventType type, const Plane plane, const unsigned faceIndex)
            : type{type}, plane{plane}, faceIndex{faceIndex} {
        }

        /**
         * Less operator used for sorting an PlaneEvent vector.
         * @param other the PlaneEvent to compare this to.
         * @return true if this should precede the other argument.
         */
        bool operator<(const PlaneEvent &other) const {
            if (this->plane.axisCoordinate == other.plane.axisCoordinate) {
                return this->type < other.type;
            }
            return this->plane.axisCoordinate < other.plane.axisCoordinate;
        }
    };

}// namespace polyhedralGravity