#pragma once

#include "polyhedralGravity/model/KDTree/KdDefinitions.h"
#include "polyhedralGravity/model/KDTree/SplitParam.h"
#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithm.h"

namespace polyhedralGravity {
    class PlaneEventAlgorithm : public PlaneSelectionAlgorithm {
    protected:
        /**
        * Generates the vector of PlaneEvents comprising all the possible candidate planes using an index list of faces. {@link PlaneEvent}
        * @param splitParam Contains the parameters of the scene to find candidate planes for. {@link SplitParam}
        * @param directions For which directions to generate the events for.
        * @return The vector of PlaneEvents.
        */
        static PlaneEventList generatePlaneEventsFromFaces(const SplitParam &splitParam, std::vector<Direction> directions);
    };
}// namespace polyhedralGravity
