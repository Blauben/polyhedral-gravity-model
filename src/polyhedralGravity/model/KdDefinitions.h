#pragma once

#include "Polyhedron.h"
#include <array>

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
 * Defines a plane that is parallel to one of the coordinate planes, by taking a point that lies on the plane and the coordinate that is fixed for every \
 * point on the plane.
 *
 * E.g. Specifying [0.0,0.0,0.0] and Direction::X would describe the YZ plane that goes through the origin.
 */
using Plane = std::pair<std::array<double, 3>, Direction>;

/**
 * Defines a rectangular box by taking two opposite corner points
 */
using Box = std::pair<std::array<double, 3>, std::array<double, 3>>;

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
    const std::vector<size_t> indexBoundFaces;
    /**
     * The current bounding box that should be divided further by the KDTree.
     */
    const Box boundingBox;
    /**
     * The direction in which the current bounding box should be divided by further.
     * @see Refer to Plane on how to interpret the Direction.
     */
    const Direction splitDirection;
};