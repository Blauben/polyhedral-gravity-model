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
     * A range of triangles on the faces vector in the KDTree. This effectively corresponds to a set of triangles. Internally iterators are constructed on demand.
     */
    struct TriangleIndexRange {
        /**
         * Start index of triangle range (inclusive).
         */
        size_t begin_idx;
        /**
         * End of triangle range (exclusive). Treated as vector_end_index = vector.size() - end_idx
         * This ensures that additional elements can be inserted
         */
        size_t end_idx;
        /**
         * The faces that connect the vertices to render the Polyhedron.
         */
        std::shared_ptr<std::vector<IndexArray3>> faces;

        /**
         * Calculates the size of the range. Corresponds to the number of bound faces.
         * @return the number of faces.
         */
        [[nodiscard]] size_t size() const {
            return end_idx - begin_idx;
        }

        /**
         * Checks for emptiness of the range.
         * @return true if the range has no bound faces.
         */
        [[nodiscard]] bool empty() const {
            return begin_idx == end_idx;
        }

        /**
         * @return The begin iterator for the bound faces.
         */
        [[nodiscard]] std::vector<IndexArray3>::iterator begin() const {
            return faces->begin() + begin_idx;
        }

        /**
        * @return The end iterator for the bound faces.
        */
        [[nodiscard]] std::vector<IndexArray3>::iterator end() const {
            return faces->begin() + end_idx;
        }

        /**
         * @return The constant begin iterator for the bound faces.
         */
        [[nodiscard]] std::vector<IndexArray3>::const_iterator cbegin() const {
            return faces->begin() + begin_idx;
        }

        /**
         * @return The constant end iterator for the bound faces.
         */
        [[nodiscard]] std::vector<IndexArray3>::const_iterator cend() const {
            return faces->begin() + end_idx;
        }

        IndexArray3 operator[](size_t idx) const {
            return faces->at(idx + begin_idx);
        }

        /**
         * Constructs a range that default initializes to include all passed faces.
         * @param faces The faces vector to base the range on.
         */
        explicit TriangleIndexRange(std::shared_ptr<std::vector<IndexArray3>> faces)
            : begin_idx{0}, end_idx{faces->size()}, faces{std::move(faces)} {
        }

        /**
         * Copies a range and assigns new boundary indices to it.
         * @param other The range to be copied.
         * @param begin New start index.
         * @param end New end index.
         */
        TriangleIndexRange(const TriangleIndexRange &other, size_t begin, size_t end) {
            this->faces = other.faces;
            this->begin_idx = begin;
            this->end_idx = end;
        }
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
        std::shared_ptr<const std::vector<Array3>> vertices;
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
         * Constructor that initializes all fields. Intended for the use with std::make_unique on KDTree root. See {@link SplitParam} fields for further information.
         *
         */
        SplitParam(std::shared_ptr<const std::vector<Array3>> vertices, std::shared_ptr<std::vector<IndexArray3>> faces, Box boundingBox, Direction splitDirection)
            : vertices{std::move(vertices)}, indexBoundFaces{TriangleIndexRange(std::move(faces))}, boundingBox{std::move(boundingBox)}, splitDirection{splitDirection} {
        }
    };
}// namespace polyhedralGravity