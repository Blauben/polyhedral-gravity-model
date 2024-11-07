#include "polyhedralGravity/model/KDTree/plane_selection/LogNPlane.h"

#include <oneapi/tbb/detail/_range_common.h>

namespace polyhedralGravity {
    // O(N*log^2(N)) implementation
    std::tuple<Plane, double, std::variant<TriangleIndexLists<2>, PlaneEventLists<2>>> LogNPlane::findPlane(const SplitParam &splitParam) {
        //initialize the default plane and make it costly
        double cost{std::numeric_limits<double>::infinity()};
        Plane optPlane{};
        bool minSide{true};
        //each vertex proposes a split plane candidate: create an event and queue it in the buffer
        PlaneEventList events{std::move(generatePlaneEvents(splitParam))};
        //records the array of Triangles MIN, MAX and PLANAR for each dimension.
        using TriangleCounter = std::array<size_t, 3>;
        using TriangleDimensionCounter = std::unordered_map<Direction, TriangleCounter>;
        //values correspond to MIN, MAX and PLANAR values in the loop for the triangle counter of a single dimension
        TriangleCounter triangleCount{0, countFaces(splitParam.boundFaces), 0};
        //initialize for all dimensions
        TriangleDimensionCounter triangleDimPosCount{{Direction::X, triangleCount}, {Direction::Y, triangleCount}, {Direction::Z, triangleCount}};
        //traverse all the events
        int i{0};
        while (i < events.size()) {
            constexpr uint8_t MIN = 0;
            constexpr uint8_t MAX = 1;
            constexpr uint8_t PLANAR = 2;
            //poll a plane to test
            Plane &candidatePlane = events[i].plane;
            //for each plane calculate the faces whose vertices lie in the plane. Differentiate between the face starting in the plane, ending in the plane or all vertices lying in the plane
            size_t p_start{0}, p_end{0}, p_planar{0};
            //count all faces that end in the plane, this works because the PlaneEvents are sorted by position and then by PlaneEventType
            while (i < events.size() && events[i].plane.orientation == candidatePlane.orientation && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::ending) {
                p_end++;
                i++;
            }
            //count all the faces that lie in the plane
            while (i < events.size() && events[i].plane.orientation == candidatePlane.orientation && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::planar) {
                p_planar++;
                i++;
            }
            //count all the faces that start in the plane
            while (i < events.size() && events[i].plane.orientation == candidatePlane.orientation && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::starting) {
                p_start++;
                i++;
            }
            //reference to the triangle counter of the current dimension -> better readability
            TriangleCounter &currentDimensionCounter{triangleDimPosCount[candidatePlane.orientation]};
            //update the absolute triangle amounts relative to the current plane using the values of the new plane
            currentDimensionCounter[PLANAR] = p_planar;
            currentDimensionCounter[MAX] -= p_planar + p_end;
            //evaluate plane and update should the new plane be more efficient
            auto [candidateCost, minSideChosen] = costForPlane(splitParam.boundingBox, candidatePlane, currentDimensionCounter[MIN], currentDimensionCounter[MAX], currentDimensionCounter[PLANAR]);
            if (candidateCost < cost) {
                cost = candidateCost;
                optPlane = candidatePlane;
                minSide = minSideChosen;
            }
            //shift the plane to the next candidate and prepare next iteration
            //TODO: review; paper incoherent here
            currentDimensionCounter[MIN] += p_planar + p_start;
            currentDimensionCounter[PLANAR] = 0;
        }
        //generate the triangle index lists for the child bounding boxes and return them along with the optimal plane and the plane's cost.
        return {optPlane, cost, generatePlaneEventSubsets(events, optPlane, minSide)};
    }


    PlaneEventList LogNPlane::generatePlaneEvents(const SplitParam &splitParam) {
        if (std::holds_alternative<TriangleIndexList>(splitParam.boundFaces)) {
            return generatePlaneEventsFromFaces(splitParam);
        }
        return std::get<PlaneEventList>(splitParam.boundFaces);
    }

    PlaneEventList LogNPlane::generatePlaneEventsFromFaces(const SplitParam &splitParam) {
        PlaneEventList events{};
        events.reserve(countFaces(splitParam.boundFaces) * 2 * 3);
        const auto &boundFaces = std::get<TriangleIndexList>(splitParam.boundFaces);
        //transform the faces into vertices
        auto [vertex3_begin, vertex3_end] = transformIterator(boundFaces.cbegin(), boundFaces.cend(), splitParam.vertices, splitParam.faces);
        std::for_each(vertex3_begin, vertex3_end, [&splitParam, &events](const auto &indexAndTriplet) {
            const auto [index, triplet] = indexAndTriplet;
            //calculate the bounding box of the face using its vertices. The edges of the box are used as candidate planes.
            const auto [minPoint, maxPoint] = Box::getBoundingBox<std::array<Array3, 3>>(triplet);
            //Generate events for each split dimension and store them in a single vector.
            for (const auto direction: {Direction::X, Direction::Y, Direction::Z}) {
                //clip plane coordinates to voxel
                const auto [minAxisCoordinate, maxAxisCoordinate] = PlaneSelectionAlgorithm::clipToVoxel(splitParam.boundingBox, direction, minPoint, maxPoint);
                // if the triangle is perpendicular to the split direction, generate a planar event with the candidate plane in which the triangle lies
                if (minAxisCoordinate == maxAxisCoordinate) {
                    events.emplace_back(
                            PlaneEventType::planar,
                            Plane{
                                    .axisCoordinate = minAxisCoordinate,
                                    .orientation = direction},
                            index);
                    return;
                }
                //else create a starting and ending event consisting of the planes defined by the min and max points of the face's bounding box.
                events.emplace_back(
                        PlaneEventType::starting,
                        Plane{
                                .axisCoordinate = minAxisCoordinate,
                                .orientation = direction},
                        index);
                events.emplace_back(
                        PlaneEventType::ending,
                        Plane{
                                .axisCoordinate = maxAxisCoordinate,
                                .orientation = direction},
                        index);
            }
        });
        //sort the events by plane position and then by PlaneEventType. Refer to {@link PlaneEventType} for the specific order
        std::sort(events.begin(), events.end());
        return events;
    }

    //TODO: continue here
    PlaneEventLists<2> LogNPlane::generatePlaneEventSubsets(const PlaneEventList &planeEvents, const Plane &plane, const bool minSide) {
        const auto faceClassification{classifyTrianglesRelativeToPlane(planeEvents, plane, minSide)};
        auto planeEventsMin = std::make_unique<PlaneEventList>();
        auto planeEventsMax = std::make_unique<PlaneEventList>();
        std::vector<PlaneEvent> planarEventsMin{}, planarEventsMax{};

        std::for_each(planeEvents.cbegin(), planeEvents.cend(), [&faceClassification, &planeEventsMin, &planeEventsMax](const auto &event) {
            switch (faceClassification[event.faceIndex]) {
                case Locale::MIN_ONLY:
                    planeEventsMin->push_back(event);
                    break;
                case Locale::MAX_ONLY:
                    planeEventsMax->push_back(event);
                    break;
                default://TODO clipping
                    switch ()
            }
        });
    }

    std::unordered_map<size_t, LogNPlane::Locale> LogNPlane::classifyTrianglesRelativeToPlane(const PlaneEventList &events, const Plane &plane, const bool minSide) {
        std::unordered_map<size_t, Locale> result{};
        //each face generates 6 plane events on average, thus the amount of faces can be roughly estimated.
        result.reserve(events.size() / 6);
        //preparing the map by initializing all faces with them having area in both sub bounding boxes
        std::for_each(events.cbegin(), events.cend(), [&result](const auto &event) {
            result[event.faceIndex] = Locale::BOTH;
        });
        //now search for conditions proving that the faces DO NOT have area in both boxes
        std::for_each(events.begin(), events.end(), [minSide, &result, &plane](const auto &event) {
            if (event.type == PlaneEventType::ending && event.plane.orientation == plane.orientation && event.plane.axisCoordinate <= plane.axisCoordinate) {
                result[event.faceIndex] = Locale::MIN_ONLY;
            } else if (event.type == PlaneEventType::starting && event.plane.orientation == plane.orientation && event.plane.axisCoordinate >= plane.axisCoordinate) {
                result[event.faceIndex] = Locale::MAX_ONLY;
            } else if (event.type == PlaneEventType::planar && event.plane.orientation == plane.orientation) {
                if (event.plane.axisCoordinate < plane.axisCoordinate || (event.plane.axisCoordinate == plane.axisCoordinate && minSide)) {
                    result[event.faceIndex] = Locale::MIN_ONLY;
                }
                if (event.plane.axisCoordinate > plane.axisCoordinate || (event.plane.axisCoordinate == plane.axisCoordinate && !minSide)) {
                    result[event.faceIndex] = Locale::MAX_ONLY;
                }
            }
        });
        return result;
    }


}// namespace polyhedralGravity