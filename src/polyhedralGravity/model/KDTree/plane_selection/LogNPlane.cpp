#include "polyhedralGravity/model/KDTree/plane_selection/LogNPlane.h"

#include <thrust/detail/minmax.h>

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
        return {optPlane, cost, generatePlaneEventSubsets(splitParam, events, optPlane, minSide)};
    }


    PlaneEventList LogNPlane::generatePlaneEvents(const SplitParam &splitParam) {
        if (std::holds_alternative<TriangleIndexList>(splitParam.boundFaces)) {
            return generatePlaneEventsFromFaces(splitParam);
        }
        return std::get<PlaneEventList>(splitParam.boundFaces);
    }

    PlaneEventList LogNPlane::generatePlaneEventsFromFaces(const SplitParam &splitParam) {
        PlaneEventList events{};
        events.reserve(countFaces(splitParam.boundFaces) * 2);
        if (std::holds_alternative<PlaneEventList>(splitParam.boundFaces)) {
            return {};
        }
        const auto &boundTriangles{std::get<TriangleIndexList>(splitParam.boundFaces)};
        //transform the faces into vertices
        auto [vertex3_begin, vertex3_end] = transformIterator(boundTriangles.cbegin(), boundTriangles.cend(), splitParam.vertices, splitParam.faces);
        std::for_each(vertex3_begin, vertex3_end, [&splitParam, &events](const auto &indexAndTriplet) {
            const auto [index, triplet] = indexAndTriplet;
            //first clip the triangles vertices to the current bounding box and then get the bounding box of the clipped triangle -> use the box edges as split plane candidates
            const auto [minPoint, maxPoint] = Box::getBoundingBox<std::vector<Array3>>(splitParam.boundingBox.clipToVoxel(triplet));
            // if the triangle is perpendicular to the split direction, generate a planar event with the candidate plane in which the triangle lies
            if (minPoint == maxPoint) {
                events.emplace_back(
                        PlaneEventType::planar,
                        Plane(minPoint, splitParam.splitDirection),
                        index);
                return;
            }
            //else create a starting and ending event consisting of the planes defined by the min and max points of the face's bounding box.
            events.emplace_back(
                    PlaneEventType::starting,
                    Plane(minPoint, splitParam.splitDirection),
                    index);
            events.emplace_back(
                    PlaneEventType::ending,
                    Plane(maxPoint, splitParam.splitDirection),
                    index);
        });
        //sort the events by plane position and then by PlaneEventType. Refer to {@link PlaneEventType} for the specific order
        std::sort(events.begin(), events.end());
        return events;
    }

    PlaneEventLists<2> LogNPlane::generatePlaneEventSubsets(const SplitParam &splitParam, const PlaneEventList &planeEvents, const Plane &plane, const bool minSide) {
        const auto faceClassification{classifyTrianglesRelativeToPlane(planeEvents, plane, minSide)};
        auto planeEventsMin = std::make_unique<PlaneEventList>();
        auto planeEventsMax = std::make_unique<PlaneEventList>();
        TriangleIndexList facesIndexBoth{};
        planeEventsMin->reserve(planeEvents.size() / 2);
        planeEventsMax->reserve(planeEvents.size() / 2);
        //value estimation taken from source paper
        facesIndexBoth.reserve(std::ceil(std::sqrt(planeEvents.size())));

        //Step 2
        std::for_each(planeEvents.cbegin(), planeEvents.cend(), [&faceClassification, &planeEventsMin, &planeEventsMax, &facesIndexBoth](const auto &event) {
            switch (faceClassification.at(event.faceIndex)) {
                //face of event only contributes to min side event can be added to side without clipping because no overlap with split plane
                case Locale::MIN_ONLY:
                    planeEventsMin->push_back(event);
                    break;
                //face of event only contributes to max side event can be added to side without clipping because no overlap with split plane
                case Locale::MAX_ONLY:
                    planeEventsMax->push_back(event);
                    break;
                //face has area on both sides -> event has to be discarded and scheduled for separate event generation
                case Locale::BOTH:
                default:
                    facesIndexBoth.push_back(event.faceIndex);
            }
        });

        auto [newMinEvents, newMaxEvents] = generatePlaneEventsForClippedFaces(splitParam, facesIndexBoth, plane);
        return {std::move(planeEventsMin), std::move(planeEventsMax)};
    }

    //Step 1
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

    //Step 3
    std::array<PlaneEventList, 2> LogNPlane::generatePlaneEventsForClippedFaces(const SplitParam &splitParam, const TriangleIndexList &faceIndices, const Plane &plane) {
        auto [minBox, maxBox] = splitParam.boundingBox.splitBox(plane);
        PlaneEventList minEvents{};
        PlaneEventList maxEvents{};
        //each face generates six new PlaneEvents and each face has area in both boxes
        minEvents.reserve(faceIndices.size() * 6);
        maxEvents.reserve(faceIndices.size() * 6);

        //lambda for creating PlaneEvents from a vertex triplet (face) in one of the two sub boxes
        const auto createPlaneEvents = [](const auto &vertices, const auto &boundingBox, const size_t faceIndex, auto &dest) {
            //clip to the voxel
            auto clipped = boundingBox.clipToVoxel(vertices);
            //create split plane anchor points using the bounding box
            const auto [minPoint, maxPoint] = Box::getBoundingBox(clipped);
            //associate parameters for PlaneEvent creation
            std::array<std::pair<const Array3, PlaneEventType>, 2> planeEventParam{
                std::make_pair(minPoint, PlaneEventType::starting),
                std::make_pair(maxPoint, PlaneEventType::ending)};
            //create planes in each dimension, be careful to cluster similar anchor points together.
            for (const auto& [point, eventType]: planeEventParam) {
                for (const auto& direction: {Direction::X, Direction::Y, Direction::Z}) {
                    dest.emplace_back(eventType, Plane(point, direction), faceIndex);
                }
            }
        };

        //transform faces to vertices
        auto [begin_it, end_it] = transformIterator(faceIndices.cbegin(), faceIndices.cend(), splitParam.vertices, splitParam.faces);
        //create new events for each face in both sub boxes
        std::for_each(begin_it, end_it, [&minBox, maxBox, &minEvents, &maxEvents, &createPlaneEvents](const auto &indexAndTriplet) {
            const auto &[index, vertexTriplet] = indexAndTriplet;
            createPlaneEvents(vertexTriplet, minBox, index, minEvents);
            createPlaneEvents(vertexTriplet, maxBox, index, maxEvents);
        });

        //sort the lists for later merge sort integration
        std::sort(minEvents.begin(), minEvents.end());
        std::sort(maxEvents.begin(), maxEvents.end());

        return {minEvents, maxEvents};
    }

}// namespace polyhedralGravity