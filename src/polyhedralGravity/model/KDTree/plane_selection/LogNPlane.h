#pragma once

#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithm.h"

#include <unordered_set>

namespace polyhedralGravity {
    class LogNPlane final : public PlaneSelectionAlgorithm {
    public:
        std::tuple<Plane, double, std::variant<TriangleIndexLists<2>, PlaneEventLists<2>>> findPlane(const SplitParam &splitParam) override;

    private:
        /**
        * Generates the vector of PlaneEvents comprising all the possible candidate planes. {@link PlaneEvent}
        * @param splitParam Contains the parameters of the scene to find candidate planes for. {@link SplitParam}
        * @return The vector of PlaneEvents
        */
        static PlaneEventList generatePlaneEvents(const SplitParam &splitParam);

        /**
        * Generates the vector of PlaneEvents comprising all the possible candidate planes using an index list of faces. {@link PlaneEvent}
        * @param splitParam Contains the parameters of the scene to find candidate planes for. {@link SplitParam}
        * @return The vector of PlaneEvents
        */
        static PlaneEventList generatePlaneEventsFromFaces(const SplitParam &splitParam);

        /**
        * When an optimal plane has been found divide the used PlaneEvents for further subdivision through child nodes.
        * @param planeEvents The events that were generated during {@link findPlane}.
        * @param plane The plane to split the faces by.
        * @param minSide Whether to include planar faces to the bounding box closer to the origin.
        * @returns The PlaneEventLists for the bounding boxes closer and further away from the origin.
        */
        static PlaneEventLists<2> generatePlaneEventSubsets(const PlaneEventList &planeEvents, const Plane &plane, bool minSide);

        enum class Locale {
            MIN_ONLY,
            MAX_ONLY,
            BOTH
        };

        /**
         * Creates a lookup table for face indices determining whether the faces has area only left of, only right of or on both sides of the box divided by the plane.
         * @param events The list of events whose faces to classify.
         * @param plane The plane tht divides the faces into two sets.
         * @param minSide
         * @return An unordered_map used for lookups of individual face locales.
         */
        static std::unordered_map<size_t, Locale> classifyTrianglesRelativeToPlane(const PlaneEventList &events, const Plane &plane, bool minSide);

        static PlaneEventList generatePlaneEventsForClippedFaces(const SplitParam &splitParam, const TriangleIndexList &faceIndices, const Plane &plane);
    };
}// namespace polyhedralGravity