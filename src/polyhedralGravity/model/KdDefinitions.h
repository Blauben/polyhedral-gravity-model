#pragma once

#include "GravityModelData.h"
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
    using Plane = std::pair<double, Direction>;

    /**
     * Defines a rectangular box by taking two opposite corner points. First is the point closest to the origin and second is the point farthest away.
     */
    using Box = std::pair<Array3, Array3>;

    /**
     * A set that stores indices of the faces vector in the KDTree. This effectively corresponds to a set of triangles. For performance purposes a std::vector is used instead of a std::set.
     */
    using TriangleIndexList = std::vector<size_t>;

    /**
    * Triangle sets contained in an array. Used by the KDTree to divide a bounding boxes included triangles into smaller subsets. For the semantic purpose of the contained sets please refer to the comments in the usage context.
     */
    template<size_t num>
    using TriangleIndexLists = std::array<std::unique_ptr<TriangleIndexList>, num>;

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
}// namespace polyhedralGravity