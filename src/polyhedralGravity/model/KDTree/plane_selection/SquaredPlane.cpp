#include "SquaredPlane.h"

namespace polyhedralGravity {

    // O(N^2) implementation
    std::tuple<Plane, double, TriangleIndexLists<2>> SquaredPlane::findPlane(const SplitParam &splitParam) {
        //initialize the default plane and make it costly
        double cost = std::numeric_limits<double>::infinity();
        Plane optPlane{0, splitParam.splitDirection};
        //store the triangleSets that are implicitly generated during plane testing for later use.
        TriangleIndexLists<2> optTriangleIndexLists{};
        //each vertex proposes a split plane candidate: test for each of them, store them in buffer set to avoid duplicate testing
        std::unordered_set<double> testedPlaneCoordinates{};
        auto [vertex3_begin, vertex3_end] = transformIterator(splitParam.indexBoundFaces.cbegin(), splitParam.indexBoundFaces.cend(), splitParam.vertices, splitParam.faces);
        std::for_each(vertex3_begin, vertex3_end, [&splitParam, &optPlane, &cost, &optTriangleIndexLists, &testedPlaneCoordinates](const auto &indexAndTriplet) {
            const auto [index, triplet] = indexAndTriplet;
            const auto [minPoint, maxPoint] = Box::getBoundingBox<std::array<Array3, 3>>(triplet);
            for (const auto planeSurfacePoint: {minPoint, maxPoint}) {
                //constructs the plane that goes through a vertex lying on the bounding box of the face to be checked and spans in a specified direction.
                Plane candidatePlane{planeSurfacePoint[static_cast<int>(splitParam.splitDirection)], splitParam.splitDirection};
                //continue if plane has already been tested
                if (testedPlaneCoordinates.find(candidatePlane.axisCoordinate) != testedPlaneCoordinates.cend()) {
                    continue;
                }
                testedPlaneCoordinates.emplace(candidatePlane.axisCoordinate);
                auto triangleIndexLists = containedTriangles(splitParam, candidatePlane);
                //evaluate the candidate plane and store if it is better than the currently stored result
                auto [candidateCost, minSideChosen] = costForPlane(splitParam.boundingBox, candidatePlane, triangleIndexLists[0]->size(), triangleIndexLists[1]->size(), triangleIndexLists[2]->size());
                if (candidateCost < cost) {
                    cost = candidateCost;
                    optPlane = candidatePlane;
                    //planar faces have to be included in one of the two sub boxes.
                    const auto &includePlanarTo = triangleIndexLists[minSideChosen ? 0 : 1];
                    includePlanarTo->insert(includePlanarTo->cend(), triangleIndexLists[2]->cbegin(), triangleIndexLists[2]->cend());
                    optTriangleIndexLists = {std::move(triangleIndexLists[0]), std::move(triangleIndexLists[1])};
                }
            }
        });
        return std::make_tuple(optPlane, cost, std::move(optTriangleIndexLists));
    }

    TriangleIndexLists<3> SquaredPlane::containedTriangles(const SplitParam &param, const Plane &split) {
        using namespace polyhedralGravity;
        //define three sets of triangles: closer to the origin, further away, in the plane
        auto index_less = std::make_unique<TriangleIndexList>(param.indexBoundFaces.size() / 2);
        auto index_greater = std::make_unique<TriangleIndexList>(param.indexBoundFaces.size() / 2);
        auto index_equal = std::make_unique<TriangleIndexList>();


        //perform check for every triangle contained in this node's bounding box.
        //transform faceIndices into the vertices
        auto [begin, end] = transformIterator(param.indexBoundFaces.cbegin(), param.indexBoundFaces.cend(), param.vertices, param.faces);
        std::for_each(begin, end, [&split, &index_greater, &index_less, &index_equal](std::pair<unsigned long, Array3Triplet> pair) {
            auto [faceIndex, vertices] = pair;
            bool less{false}, greater{false};
            for (const Array3 vertex: vertices) {
                //vertex is closer to the origin than the plane
                if (vertex[static_cast<int>(split.orientation)] < split.axisCoordinate && !less) {
                    less = true;
                    //triangle has area in the closer bounding box and needs to be checked there for intersections
                    index_less->push_back(faceIndex);
                }
                //vertex is farther away of the origin than the plane
                else if (vertex[static_cast<int>(split.orientation)] > split.axisCoordinate && !greater) {
                    greater = true;
                    //triangle has area in the greater bounding box and needs to be checked there for intersections
                    index_greater->push_back(faceIndex);
                }
            }
            //all vertices of the triangle lie in the plane -> triangle lies in the plane
            if (!less && !greater) {
                index_equal->push_back(faceIndex);
            }
        });
        return std::array{(std::move(index_less)), (std::move(index_greater)), (std::move(index_equal))};
    }
}// namespace polyhedralGravity