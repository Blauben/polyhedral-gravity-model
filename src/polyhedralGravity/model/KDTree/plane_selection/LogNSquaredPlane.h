#pragma once

#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithm.h"

#include <unordered_set>

namespace polyhedralGravity {
    class LogNSquaredPlane final : public PlaneSelectionAlgorithm {
    public:
        std::tuple<Plane, double, TriangleIndexLists<2>> findPlane(const SplitParam &splitParam) override;

    private:
        /**
         * Generates the optimal split plane considering a single dimension.
         * @param splitParam Specifies the parameters needed to perform the splits.
         * @return the optimal plane, its cost, the events that were generated in the process, and whether to include planar triangles in the minimal bounding box.
         */
        static std::tuple<Plane, double, std::vector<PlaneEvent>, bool> findPlaneForSingleDimension(const SplitParam &splitParam);
        /**
        * Generates the vector of PlaneEvents comprising all the possible candidate planes. {@link PlaneEvent}
        * @param splitParam Contains the parameters of the scene to find candidate planes for. {@link SplitParam}
        * @return The vector of PlaneEvents
        */
        static std::vector<PlaneEvent> generatePlaneEvents(const SplitParam &splitParam);

        /**
        * When an optimal plane has been found extract the index lists of faces for further subdivision through child nodes.
        * @param planeEvents The events that were generated during {@link findPlane}.
        * @param plane The plane to split the faces by.
        * @param minSide Whether to include planar faces to the bounding box closer to the origin.
        * @return The triangleIndexlists for the bounding boxes closer and further away from the origin.
        */
        static TriangleIndexLists<2> generateTriangleSubsets(const std::vector<PlaneEvent> &planeEvents, const Plane &plane, bool minSide);
    };

}// namespace polyhedralGravity