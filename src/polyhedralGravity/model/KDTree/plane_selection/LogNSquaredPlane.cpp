#include "polyhedralGravity/model/KDTree/plane_selection/LogNSquaredPlane.h"

namespace polyhedralGravity {
    // O(N*log^2(N)) implementation
    std::tuple<Plane, double, TriangleIndexLists<2>> LogNSquaredPlane::findPlane(const SplitParam &splitParam) {//TODO iterate over all dimensions
        //initialize the default plane and make it costly
        double cost = std::numeric_limits<double>::infinity();
        Plane optPlane{};
        bool minSide{true};
        //each vertex proposes a split plane candidate: create an event and queue it in the buffer
        std::vector<PlaneEvent> events{std::move(generatePlaneEvents(splitParam))};
        size_t trianglesMin{0}, trianglesMax{splitParam.indexBoundFaces.size()}, trianglesPlanar{0};
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
        //generate the triangle index lists for the child bounding boxes and return them along with the optimal plane and the plane's cost.
        return std::make_tuple(optPlane, cost, generateTriangleSubsets(events, optPlane, minSide));
    }

    std::vector<PlaneEvent> LogNSquaredPlane::generatePlaneEvents(const SplitParam &splitParam) {
        std::vector<PlaneEvent> events{};
        events.reserve(splitParam.indexBoundFaces.size() * 2);
        //transform the faces into vertices
        auto [vertex3_begin, vertex3_end] = KDTree::transformIterator(splitParam.indexBoundFaces.cbegin(), splitParam.indexBoundFaces.cend(), splitParam.vertices, splitParam.faces);
        std::for_each(vertex3_begin, vertex3_end, [&splitParam, &events](const auto &indexAndTriplet) {
            const auto [index, triplet] = indexAndTriplet;
            //calculate the bounding box of the face using its vertices. The edges of the box are used as candidate planes.
            const auto [minPoint, maxPoint] = getBoundingBox<std::array<Array3, 3>>(triplet);
            //clip plane coordinates to voxel
            const auto [minAxisCoordinate, maxAxisCoordinate] = clipToVoxel(splitParam.boundingBox, splitParam.splitDirection, minPoint, maxPoint);
            // if the triangle is perpendicular to the split direction, generate a planar event with the candidate plane in which the triangle lies
            if (minAxisCoordinate == maxAxisCoordinate) {
                events.emplace_back(
                        PlaneEventType::planar,
                        Plane{
                                .axisCoordinate = minAxisCoordinate,
                                .orientation = splitParam.splitDirection},
                        index);
                return;
            }
            //else create a starting and ending event consisting of the planes defined by the min and max points of the face's bounding box.
            events.emplace_back(
                    PlaneEventType::starting,
                    Plane{
                            .axisCoordinate = minAxisCoordinate,
                            .orientation = splitParam.splitDirection},
                    index);
            events.emplace_back(
                    PlaneEventType::ending,
                    Plane{
                            .axisCoordinate = maxAxisCoordinate,
                            .orientation = splitParam.splitDirection},
                    index);
        });
        //sort the events by plane position and then by PlaneEventType. Refer to {@link PlaneEventType} for the specific order
        std::sort(events.begin(), events.end());
        return events;
    }

    TriangleIndexLists<2> LogNSquaredPlane::generateTriangleSubsets(const std::vector<PlaneEvent> &planeEvents, const Plane &plane, const bool minSide) {
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