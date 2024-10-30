#pragma once

#include "polyhedralGravity/model/GravityModelData.h"

#include <array>
#include <memory>
#include <utility>
#include <vector>

namespace polyhedralGravity {
    /**
     * Assigns an integer index to the coordinate axes
     *
     * Used to specify coordinates. E.g. CoordinateArray[Direction]
     */
    enum class Direction {
        X = 0,
        Y = 1,
        Z = 2
    };

    /**
     * Number of dimensions for the polyhedron. Also corresponds to the number of elements of the {@link Direction} enum.
     */
    constexpr int DIMENSIONS = 3;

    /**
     * Defines a plane that is parallel to one of the coordinate planes, by taking the fixed axis coordinate value for the plane and the coordinate index ({@link Direction}) that is fixed for every \
     * point on the plane.
     *
     * E.g. Specifying 0.0 and Direction::X would describe the YZ plane that goes through the origin.
     */
    struct Plane {
        /**
         * Each point lying on the plane has to have this value in the dimension specified in the orientation parameter.
         */
        double axisCoordinate;
        /**
         * Specifies which coordinate dimension is fixed for every point on the plane.
         */
        Direction orientation;
    };

    /**
     * Defines a rectangular box by taking two opposite corner points. First is the point closest to the origin and second is the point farthest away.
     */
    struct Box {
        /**
         * The point closer to the origin, thus minimal
         */
        Array3 minPoint;
        /**
         * The point further away from the origin, thus maximal.
         */
        Array3 maxPoint;

        /**
         * Calculates the intersection points of a ray and a box.
         * @param origin The origin of the ray.
         * @param ray The ray direction vector.
         * @return Parameters t of the equation $ intersection_point = origin + t * ray $ for the entry and exit intersection points.
         */
        [[nodiscard]] std::pair<double, double> rayBoxIntersection(const Array3 &origin, const Array3 &ray) const {
            //calculate the parameter t in $ origin + t * ray = point $
            auto const lambdaIntersectSlabPoint = [&origin, &ray](const Array3 &point) {
                return std::make_tuple(
                        (point[0] - origin[0]) / ray[0],
                        (point[1] - origin[1]) / ray[1],
                        (point[2] - origin[2]) / ray[2]);
            };
            //intersections with slabs defined through minPoint
            auto [tx_1, ty_1, tz_1] = lambdaIntersectSlabPoint(minPoint);
            //intersections with slabs defined through maxPoint
            auto [tx_2, ty_2, tz_2] = lambdaIntersectSlabPoint(maxPoint);

            //return the parameters in ordered by '<'
            auto [tx_enter, tx_exit] = std::minmax(tx_1, tx_2);
            auto [ty_enter, ty_exit] = std::minmax(ty_1, ty_2);
            auto [tz_enter, tz_exit] = std::minmax(tz_1, tz_2);

            //calculate the point where all slabs have been entered: t_enter
            const double t_enter{std::max(tx_enter, std::max(ty_enter, tz_enter))};
            //calculates the point where the first slab has been exited: t_exit
            const double t_exit{std::min(tx_exit, std::min(ty_exit, tz_exit))};

            return {t_enter, t_exit};
        }

        /**
        * Calculates the surface area of a box.
        * @return the surface area
        */
        [[nodiscard]] double surfaceArea() const {
            const double width = std::abs(maxPoint[0] - minPoint[0]);
            const double length = std::abs(maxPoint[1] - minPoint[1]);
            const double height = std::abs(maxPoint[2] - minPoint[2]);
            return 2 * (width * length + width * height + length * height);
        }

        /**
       * Splits this box into two new boxes.
       * @param plane the plane by which to split the original box.
       * @return a pair of boxes that result by splitting this box.
       */
        [[nodiscard]] std::pair<Box, Box> splitBox(const Plane &plane) const {
            //clone the original box two times -> modify clones to become child boxes defined by the splitting plane
            Box box1{*this};
            Box box2{*this};
            const Direction &axis{plane.orientation};
            //Shift edges of the boxes to match the plane
            box1.maxPoint[static_cast<int>(axis)] = plane.axisCoordinate;
            box2.minPoint[static_cast<int>(axis)] = plane.axisCoordinate;
            return std::make_pair(box1, box2);
        }

        /**
        * Finds the minimal bounding box for a set of vertices.
        * @param vertices the set of vertex coordinates for which to find the box
        * @return the bounding box {@link Box}
        */
        template<typename Container>
        static Box getBoundingBox(const Container &vertices) {
            using namespace util;
            return Box(findMinMaxCoordinates<Container, Array3>(vertices));
        }

        explicit Box(const std::pair<Array3, Array3> &pair)
            : minPoint{pair.first}, maxPoint{pair.second} {
        }
        Box()
            : minPoint{0.0, 0.0, 0.0}, maxPoint{0.0, 0.0, 0.0} {
        }
    };

    /**
     * A set that stores indices of the faces vector in the KDTree. This effectively corresponds to a set of triangles. For performance purposes a std::vector is used instead of a std::set.
     */
    using TriangleIndexList = std::vector<size_t>;

    /**
    * Triangle sets contained in an array. Used by the KDTree to divide a bounding boxes included triangles into smaller subsets. For the semantic purpose of the contained sets please refer to the comments in the usage context.
     */
    template<size_t num>
    using TriangleIndexLists = std::array<std::unique_ptr<TriangleIndexList>, num>;

    //forward declaration for SplitParam
    class PlaneSelectionAlgorithm;

    /**
     * Helper struct to bundle important parameters required for splitting a Polyhedron for better readability.
     */
    struct SplitParam {
        /**
         * The vertices that compose the Polyhedron.
         */
        const std::vector<Array3> &vertices;
        /**
         * The faces that connect the vertices to render the Polyhedron.
         */
        const std::vector<IndexArray3> &faces;
        /**
         * An index list of faces that are included in the current bounding box of the KDTree. Important when building deeper levels of a KDTree.
         */
        TriangleIndexList indexBoundFaces;
        /**
         * The current bounding box that should be divided further by the KDTree.
         */
        Box boundingBox;
        /**
         * The direction in which the current bounding box should be divided by further.
         * Refer to {@link Plane} on how to interpret the Direction.
         */
        Direction splitDirection;

        /**
         * The algorithm used to find optimal split planes.
         */
        std::unique_ptr<PlaneSelectionAlgorithm> planeSelectionStrategy;

        /**
         * Constructor that initializes all fields. Intended for the use with std::make_unique. See {@link SplitParam} fields for further information.
         *
         */
        SplitParam(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, const Box &boundingBox, const Direction splitDirection, std::unique_ptr<PlaneSelectionAlgorithm> algorithm)
            : vertices{vertices}, faces{faces}, indexBoundFaces{TriangleIndexList(faces.size())}, boundingBox{boundingBox}, splitDirection{splitDirection}, planeSelectionStrategy{std::move(algorithm)} {
            std::iota(indexBoundFaces.begin(), indexBoundFaces.end(), 0);
        };
    };

}// namespace polyhedralGravity