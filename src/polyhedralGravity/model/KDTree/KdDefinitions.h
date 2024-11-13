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

        //refer to https://en.wikipedia.org/wiki/Slab_method
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
    template<size_t Number>
    using TriangleIndexLists = std::array<std::unique_ptr<TriangleIndexList>, Number>;

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
         * Constructor that initializes all fields. Intended for the use with std::make_unique. See {@link SplitParam} fields for further information.
         *
         */
        SplitParam(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, Box boundingBox, Direction splitDirection)
            : vertices{vertices}, faces{faces}, indexBoundFaces{TriangleIndexList(faces.size())}, boundingBox{std::move(boundingBox)}, splitDirection{splitDirection} {
            std::iota(indexBoundFaces.begin(), indexBoundFaces.end(), 0);
        }
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
        unsigned int faceIndex;

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