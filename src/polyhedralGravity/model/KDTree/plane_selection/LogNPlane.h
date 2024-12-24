#pragma once

#include "polyhedralGravity/model/KDTree/SplitParam.h"
#include "polyhedralGravity/model/KDTree/plane_selection/PlaneEventAlgorithm.h"

#include <unordered_set>

namespace polyhedralGravity {
    class LogNPlane final : public PlaneEventAlgorithm {
    public:
        std::tuple<Plane, double, std::variant<TriangleIndexVectors<2>, PlaneEventVectors<2>>> findPlane(const SplitParam &splitParam) override;

    private:
        /**
        * Generates the vector of PlaneEvents comprising all the possible candidate planes. {@link PlaneEvent}
        * @param splitParam Contains the parameters of the scene to find candidate planes for. {@link SplitParam}
        * @return The vector of PlaneEvents
        */
        static PlaneEventVector generatePlaneEvents(const SplitParam &splitParam);

        /**
        * When an optimal plane has been found divide the used PlaneEvents for further subdivision through child nodes.
        * @param splitParam Contains information about the current scene to be split.
        * @param planeEvents The events that were generated during {@link findPlane}.
        * @param plane The plane to split the faces by.
        * @param minSide Whether to include planar faces to the bounding box closer to the origin.
        * @returns The PlaneEventLists for the bounding boxes closer and further away from the origin.
        */
        static PlaneEventVectors<2> generatePlaneEventSubsets(const SplitParam &splitParam, const PlaneEventVector &planeEvents, const Plane &plane, bool minSide);

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
        static std::unordered_map<size_t, Locale> classifyTrianglesRelativeToPlane(const PlaneEventVector &events, const Plane &plane, bool minSide);

        /**
        * Creates new events for two sub bounding boxes out of faces that overlap both of them.
        * @param splitParam Contains information about the current scene to be split.
        * @param faceIndices The index list of faces that straddle the plane.
        * @param plane The plane that splits the scene's bounding box into two new sub boxes.
        * @return Two new PlaneEventLists for the minimal and maximal bounding boxes respectively (unsorted!).
        */
        static std::array<PlaneEventVector, 2> generatePlaneEventsForClippedFaces(const SplitParam &splitParam, const TriangleIndexVector &faceIndices, const Plane &plane);

        /**
         * Takes two sorted PlaneEventLists and merges them in a single merge sort step.
         * @param first The first PlaneEventList.
         * @param second The second PlaneEventList
         * @return A unique_ptr to a combined sorted PlaneEventList.
         */
        static std::unique_ptr<PlaneEventVector> mergePlaneEventLists(const PlaneEventVector &first, const PlaneEventVector &second);
    };
}// namespace polyhedralGravity