#pragma once

#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithm.h"

#include <unordered_set>

namespace polyhedralGravity {
    /**
* O(N^2) implementation to finding optimal split planes.
*/
    class SquaredPlane : public PlaneSelectionAlgorithm {
        /**
        * Finds the optimal split plane to split a provided rectangle section optimally.
        * @param splitParam specifies the polyhedron section to be split @link SplitParam.
        * @return Tuple of the optimal plane to split the specified bounding box, its cost as double and a list of triangle sets with respective positions to the found plane. Refer to {@link TriangleIndexLists<2>} for more information.
        */
        std::tuple<Plane, double, TriangleIndexLists<2>> findPlane(const SplitParam &splitParam) override;

        /**
        * Splits a section of a polyhedron into two bounding boxes and calculates the triangle face sets contained in the new bounding boxes.
        * @param param specifies the polyhedron section to be split.
        * @param split the plane by which to split the polyhedron section.
        * @return Three triangle lists contained in an array. Those being the set of triangles with non-zero area in the bounding box closer to the origin with respect to the split plane,
        * the set of triangles with non-zero area in the bounding box further away from the origin with respect to the split plane.
        * The set of triangles that lies on the plane.
        */
        static TriangleIndexLists<3> containedTriangles(const SplitParam &param, const Plane &split);
    };
}