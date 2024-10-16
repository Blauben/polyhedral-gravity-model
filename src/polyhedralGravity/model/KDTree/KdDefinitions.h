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
    using Plane = std::pair<double, Direction>;

    /**
     * Defines a rectangular box by taking two opposite corner points. First is the point closest to the origin and second is the point farthest away.
     */
    using Box = std::pair<Array3, Array3>;

    /**
     * A range of triangles on the faces vector in the KDTree. This effectively corresponds to a set of triangles. Internally iterators begin() and end() are stored in a pair.
     */
    struct TriangleIndexRange {
        std::vector<IndexArray3> &faces;
        size_t begin_idx;
        size_t end_idx;

        [[nodiscard]] size_t size() const {
            return end_idx - begin_idx;
        }

        [[nodiscard]] bool empty() const {
            return begin_idx == end_idx;
        }

        std::vector<IndexArray3>::iterator begin() {
            return faces.begin() + begin_idx;
        }

        std::vector<IndexArray3>::iterator end() {
            return faces.begin() + end_idx;
        }

        TriangleIndexRange(size_t begin, size_t end, std::vector<IndexArray3> &faces)
            : faces{faces}, begin_idx{begin}, end_idx{end} {
        }
        TriangleIndexRange(TriangleIndexRange& other) = default;
        TriangleIndexRange & operator=(const TriangleIndexRange &other) = default;
    };

    /**
    * Triangle ranges contained in an array. Used by the KDTree to divide a bounding boxes included triangles into smaller subsets. For the semantic purpose of the contained ranges please refer to the comments in the usage context.
     */
    template<int num>
    using TriangleIndexRanges = std::array<TriangleIndexRange, num>;

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
        std::vector<IndexArray3> &faces;
        /**
         * An index range of faces that are included in the current bounding box of the KDTree. Important when building deeper levels of a KDTree.
         */
        TriangleIndexRange indexBoundFaces;
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
        SplitParam(const std::vector<Array3> &vertices, std::vector<IndexArray3> &faces, TriangleIndexRange indexBoundFaces, Box boundingBox, Direction splitDirection)
            : vertices{vertices}, faces{faces}, indexBoundFaces{indexBoundFaces}, boundingBox{std::move(boundingBox)}, splitDirection{splitDirection} {
        }
    };
}// namespace polyhedralGravity