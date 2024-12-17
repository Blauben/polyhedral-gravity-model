#include "polyhedralGravity/model/KDTree/plane_selection/LogNSquaredPlane.h"

namespace polyhedralGravity {
    // O(N*log^2(N)) implementation
    std::tuple<Plane, double, std::variant<TriangleIndexLists<2>, PlaneEventLists<2>>> LogNSquaredPlane::findPlane(const SplitParam &splitParam) {
        Plane optPlane{};
        double cost{std::numeric_limits<double>::infinity()};
        PlaneEventList optimalEvents{};
        bool minSide{true};
        for (const auto dimension: {Direction::X, Direction::Y, Direction::Z}) {
            splitParam.splitDirection = dimension;
            auto [candidatePlane, candidateCost, events, minSideChosen] = findPlaneForSingleDimension(splitParam);
            if (candidateCost < cost) {
                optPlane = candidatePlane;
                cost = candidateCost;
                optimalEvents = events;
                minSide = minSideChosen;
            }
        }
        //generate the triangle index lists for the child bounding boxes and return them along with the optimal plane and the plane's cost.
        return {optPlane, cost, generateTriangleSubsets(optimalEvents, optPlane, minSide)};
    }

    std::tuple<Plane, double, PlaneEventList, bool> LogNSquaredPlane::findPlaneForSingleDimension(const SplitParam &splitParam) {
        //initialize the default plane and make it costly
        double cost{std::numeric_limits<double>::infinity()};
        Plane optPlane{};
        bool minSide{true};
        //each vertex proposes a split plane candidate: create an event and queue it in the buffer
        PlaneEventList events{std::move(generatePlaneEventsFromFaces(splitParam, {splitParam.splitDirection}))};
        size_t trianglesMin{0}, trianglesMax{countFaces(splitParam.boundFaces)}, trianglesPlanar{0};
        //traverse all the events
        int i{0};
        while (i < events.size()) {
            //poll a plane to test
            Plane &candidatePlane = events[i].plane;
            //for each plane calculate the faces whose vertices lie in the plane. Differentiate between the face starting in the plane, ending in the plane or all vertices lying in the plane
            size_t p_start{0}, p_end{0}, p_planar{0};
            //count all faces that end in the plane, this works because the PlaneEvents are sorted by position and then by PlaneEventType
            while (i < events.size() && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::ending) {
                p_end++;
                i++;
            }
            //count all the faces that lie in the plane
            while (i < events.size() && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::planar) {
                p_planar++;
                i++;
            }
            //count all the faces that start in the plane
            while (i < events.size() && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::starting) {
                p_start++;
                i++;
            }
            //update the absolute triangle amounts relative to the current plane using the values of the new plane
            trianglesPlanar = p_planar;
            trianglesMax -= p_planar + p_end;
            //evaluate plane and update should the new plane be more efficient
            auto [candidateCost, minSideChosen] = costForPlane(splitParam.boundingBox, candidatePlane, trianglesMin, trianglesMax, trianglesPlanar);
            if (candidateCost < cost) {
                cost = candidateCost;
                optPlane = candidatePlane;
                minSide = minSideChosen;
            }
            //shift the plane to the next candidate and prepare next iteration
            trianglesMin += p_planar + p_start;
            trianglesPlanar = 0;
        }
        return {optPlane, cost, events, minSide};
    }


    TriangleIndexLists<2> LogNSquaredPlane::generateTriangleSubsets(const PlaneEventList &planeEvents, const Plane &plane, const bool minSide) {
        auto facesMin = std::make_unique<TriangleIndexList>();
        auto facesMax = std::make_unique<TriangleIndexList>();
        //set data structure to avoid processing faces twice -> introduces O(1) lookup instead of O(n) lookup using the vectors directly
        std::unordered_set<size_t> facesMinLookup{};
        std::unordered_set<size_t> facesMaxLookup{};
        //each face will most of the time generate two events, the split plane will try to distribute the faces evenly
        //Thus reserving 0.5 * 0.5 * planeEvents.size() for each vector
        facesMin->reserve(planeEvents.size() / 4);
        facesMax->reserve(planeEvents.size() / 4);
        facesMinLookup.reserve(planeEvents.size() / 4);
        facesMaxLookup.reserve(planeEvents.size() / 4);
        std::for_each(planeEvents.cbegin(), planeEvents.cend(), [&facesMin, &facesMax, &plane, minSide, &facesMinLookup, &facesMaxLookup](const auto &event) {
            //lambda function to combine lookup and insertion into one place
            auto insertIfAbsent = [&facesMin, &facesMinLookup, &facesMax, &facesMaxLookup](const size_t faceIndex, const uint8_t index) {
                const auto &vector = index == MIN ? facesMin : facesMax;
                auto &lookup = index == MIN ? facesMinLookup : facesMaxLookup;
                if (lookup.find(faceIndex) == lookup.end()) {
                    lookup.insert(faceIndex);
                    vector->push_back(faceIndex);
                }
                // Since each face can only be referenced by max two events (since there are only two planes encasing it),
                // after the face has been already been processed once, it can be removed from the lookup buffer after the second time to save space
                else {
                    lookup.erase(lookup.find(faceIndex));
                }
            };
            //sort the triangles by inferring their position from the event's candidate split plane
            if (event.plane.axisCoordinate != plane.axisCoordinate) {
                insertIfAbsent(event.faceIndex, event.plane.axisCoordinate < plane.axisCoordinate ? MIN : MAX);
            }
            //the triangle is in, starting or ending in the plane to split by -> the PlanarEventType signals its position then
            else if (event.type == PlaneEventType::planar) {
                //minSide specifies where to include planar faces
                insertIfAbsent(event.faceIndex, minSide ? MIN : MAX);
            }
            //the face starts in the plane, thus its area overlaps with the bounding box further away from the origin.
            else if (event.type == PlaneEventType::starting) {
                insertIfAbsent(event.faceIndex, MAX);
            }
            //the face ends in the plane, thus its area overlaps with the bounding box closer to the origin.
            else {
                insertIfAbsent(event.faceIndex, MIN);
            }
        });
        return {std::move(facesMin), std::move(facesMax)};
    }

}// namespace polyhedralGravity