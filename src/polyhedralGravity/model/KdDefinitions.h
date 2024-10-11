#pragma once

#include "GravityModelData.h"
#include <array>
#include <memory>
#include <vector>

/**
 * Assigns an integer index to the coordinate axes
 *
 * Used to specify coordinates. E.g CoordinateArray[Direction]
 */
enum Direction {
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
 * Defines a rectangular box by taking two opposite corner points
 */
using Box = std::pair<std::array<double, 3>, std::array<double, 3>>;

/**
* Three triangle sets contained in an array. Those being the set of triangles with non-zero area in the bounding box closer to the origin with respect to the split plane,
     * the set of triangles with non-zero area in the bounding box further away from the origin with respect to the split plane
     * and the set of triangles that overlap with the split plane itself.
 */
using TriangleIndexLists = std::array<std::unique_ptr<std::vector<size_t>>, 3>;

/**
 * Helper struct to bundle important parameters required for splitting a Polyhedron for better readability
 */
struct SplitParam {
    /**
     * The vertices that compose the Polyhedron
     */
    const std::vector<polyhedralGravity::Array3> &vertices;
    /**
     * The faces that connect the vertices to render the Polyhedron
     */
    const std::vector<polyhedralGravity::IndexArray3> &faces;
    /**
     * An index list of faces that are included in the current bounding box of the KDTree. Important when building deeper levels of a KDTree.
     */
    std::vector<size_t> indexBoundFaces;
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
    SplitParam(const std::vector<polyhedralGravity::Array3> &vertices, const std::vector<polyhedralGravity::IndexArray3> &faces, std::vector<size_t> &indexBoundFaces, Box boundingBox, Direction splitDirection)
        : vertices{vertices}, faces{faces}, indexBoundFaces{indexBoundFaces}, boundingBox{std::move(boundingBox)}, splitDirection{splitDirection} {
    }
    SplitParam(const SplitParam &) = default;
    SplitParam(SplitParam &&) = delete;
};