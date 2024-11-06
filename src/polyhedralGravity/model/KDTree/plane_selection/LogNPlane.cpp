#include "polyhedralGravity/model/KDTree/plane_selection/LogNPlane.h"

#include <oneapi/tbb/detail/_range_common.h>

namespace polyhedralGravity {
    // O(N*log^2(N)) implementation
    std::tuple<Plane, double, TriangleIndexLists<2>> LogNPlane::findPlane(const SplitParam &splitParam) {
        //initialize the default plane and make it costly
        double cost{std::numeric_limits<double>::infinity()};
        Plane optPlane{};
        bool minSide{true};
        //each vertex proposes a split plane candidate: create an event and queue it in the buffer
        std::vector<PlaneEvent> events{std::move(generatePlaneEvents(splitParam))};
        //records the array of Triangles MIN, MAX and PLANAR for each dimension.
        using TriangleCounter = std::array<size_t, 3>;
        using TriangleDimensionCounter = std::unordered_map<Direction, TriangleCounter>;
        //values correspond to MIN, MAX and PLANAR values in the loop for the triangle counter of a single dimension
        TriangleCounter triangleCount{0, splitParam.indexBoundFaces.size(), 0};
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
        return {optPlane, cost, generateTriangleSubsets(events, optPlane, minSide)};
    }


    std::vector<PlaneEvent> LogNPlane::generatePlaneEvents(const SplitParam &splitParam) {
        std::vector<PlaneEvent> events{};
        events.reserve(splitParam.indexBoundFaces.size() * 2 * 3);
        //transform the faces into vertices
        auto [vertex3_begin, vertex3_end] = transformIterator(splitParam.indexBoundFaces.cbegin(), splitParam.indexBoundFaces.cend(), splitParam.vertices, splitParam.faces);
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
    TriangleIndexLists<2> LogNPlane::generateTriangleSubsets(const std::vector<PlaneEvent> &planeEvents, const Plane &plane, const bool minSide) {
        auto facesMin = std::make_unique<TriangleIndexList>();
        auto facesMax = std::make_unique<TriangleIndexList>();
        //set data structure to avoid processing faces twice -> introduces O(1) lookup instead of O(n) lookup using the vectors directly
        std::unordered_set<unsigned long> facesMinLookup{};
        std::unordered_set<unsigned long> facesMaxLookup{};
        //each face will most of the time generate two events, the split plane will try to distribute the faces evenly
        //Thus reserving 0.5 * 0.5 * planeEvents.size() for each vector
        facesMin->reserve(planeEvents.size() / 4);
        facesMax->reserve(planeEvents.size() / 4);
        facesMinLookup.reserve(planeEvents.size() / 4);
        facesMaxLookup.reserve(planeEvents.size() / 4);
        std::for_each(planeEvents.cbegin(), planeEvents.cend(), [&facesMin, &facesMax, &plane, minSide, &facesMinLookup, &facesMaxLookup](const auto &event) {
            //lambda function to combine lookup and insertion into one place
            auto insertIfAbsent = [&facesMin, &facesMinLookup, &facesMax, &facesMaxLookup](const unsigned long faceIndex, const u_int8_t index) {
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