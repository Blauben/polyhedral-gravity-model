#include "polyhedralGravity/model/KDTree/KDTree.h"

namespace polyhedralGravity {

    //on initialization of the tree a single bounding box which includes all the faces of the polyhedron is generated. Both the list of included faces and the parameters of the box are written to the split parameters
    KDTree::KDTree(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces)
        : _vertices{vertices}, _faces{faces}, _splitParam{std::make_unique<SplitParam>(_vertices, _faces, getBoundingBox(vertices), Direction::X)} {
    }

    TreeNode &KDTree::getRootNode() {
        //if the node has already been generated, don't do it again. Instead let the factory determine the TreeNode subclass based on the optimal split.
        if (!this->_rootNode) {
            this->_rootNode = TreeNodeFactory::treeNodeFactory(*std::move(this->_splitParam));
        }
        return *this->_rootNode;
    }

    size_t KDTree::countIntersections(const Array3 &origin, const Array3 &ray) {
        //it's possible that a single intersection point is on the edge between two triangles. The point would be counted twice if the intersection points were not documented -> use of std::set
        std::set<Array3> set{};
        this->getFaceIntersections(origin, ray, set);
        return set.size();
    }

    void KDTree::getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) {
        this->getRootNode().getFaceIntersections(origin, ray, intersections);
    }

    // O(N*log^2(N)) implementation
    std::tuple<Plane, double, TriangleIndexLists<2>> KDTree::findPlane(const SplitParam &splitParam) {//TODO iterate over all dimensions
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

    std::vector<PlaneEvent> KDTree::generatePlaneEvents(const SplitParam &splitParam) {
        std::vector<PlaneEvent> events{};
        events.reserve(splitParam.indexBoundFaces.size() * 2);
        //transform the faces into vertices
        auto [vertex3_begin, vertex3_end] = transformIterator(splitParam.indexBoundFaces.cbegin(), splitParam.indexBoundFaces.cend(), splitParam.vertices, splitParam.faces);
        std::for_each(vertex3_begin, vertex3_end, [&splitParam, &events](const auto &indexAndTriplet) {
            const auto [index, triplet] = indexAndTriplet;
            //calculate the bounding box of the face using its vertices. The edges of the box are used as candidate planes.
            const auto [minPoint, maxPoint] = getBoundingBox<std::array<Array3, 3>>(triplet);
            // if the triangle is perpendicular to the split direction, generate a planar event with the candidate plane in which the triangle lies
            if (minPoint[static_cast<int>(splitParam.splitDirection)] == maxPoint[static_cast<int>(splitParam.splitDirection)]) {
                events.emplace_back(
                        PlaneEventType::planar,
                        Plane{
                                .axisCoordinate = minPoint[static_cast<int>(splitParam.splitDirection)],
                                .orientation = splitParam.splitDirection},
                        index);
                return;
            }
            //else create a starting and ending event consisting of the planes defined by the min and max points of the face's bounding box.
            events.emplace_back(
                    PlaneEventType::starting,
                    Plane{
                            .axisCoordinate = minPoint[static_cast<int>(splitParam.splitDirection)],
                            .orientation = splitParam.splitDirection},
                    index);
            events.emplace_back(
                    PlaneEventType::ending,
                    Plane{
                            .axisCoordinate = maxPoint[static_cast<int>(splitParam.splitDirection)],
                            .orientation = splitParam.splitDirection},
                    index);
        });
        //sort the events by plane position and then by PlaneEventType. Refer to {@link PlaneEventType} for the specific order
        std::sort(events.begin(), events.end());
        return events;
    }

    TriangleIndexLists<2> KDTree::generateTriangleSubsets(const std::vector<PlaneEvent> &planeEvents, const Plane &plane, const bool minSide) {
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


    template<typename Container>
    Box KDTree::getBoundingBox(const Container &vertices) {
        using namespace util;
        return Box(findMinMaxCoordinates<Container, Array3>(vertices));
    }

    std::pair<Box, Box> KDTree::splitBox(const Box &box, const Plane &plane) {
        //clone the original box two times -> modify clones to become child boxes defined by the splitting plane
        Box box1{box};
        Box box2{box};
        const Direction &axis{plane.orientation};
        //Shift edges of the boxes to match the plane
        box1.maxPoint[static_cast<int>(axis)] = plane.axisCoordinate;
        box2.minPoint[static_cast<int>(axis)] = plane.axisCoordinate;
        return std::make_pair(box1, box2);
    }

    std::pair<const double, bool> KDTree::costForPlane(const Box &boundingBox, const Plane &plane, const size_t trianglesMin, const size_t trianglesMax, const size_t trianglesPlanar) {
        //Checks if the split plane is one of the faces of the bounding box, if so the split is useless
        if (plane.axisCoordinate == boundingBox.minPoint[static_cast<int>(plane.orientation)] || plane.axisCoordinate == boundingBox.maxPoint[static_cast<int>(plane.orientation)]) {
            //will be discarded later because not splitting is cheaper (finitely many nodes!) than using this plane (infinite cost)
            return {std::numeric_limits<double>::infinity(), false};
        }
        //calculate parameters for Surface Area Heuristic (SAH): childBoxSurfaceAreas; number of contained triangles in each box
        auto [box1, box2] = splitBox(boundingBox, plane);
        //equalT are triangles lying in the plane (not in the boxes)
        const double surfaceAreaBounding = surfaceAreaOfBox(boundingBox);
        const double surfaceArea1 = surfaceAreaOfBox(box1);
        const double surfaceArea2 = surfaceAreaOfBox(box2);
        //evaluate SAH: Include equalT once in each box and record option with minimum cost
        const double costLesser = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * (static_cast<double>(trianglesMin + trianglesPlanar)) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>(trianglesMax));
        const double costUpper = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * static_cast<double>(trianglesMin) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>((trianglesMax + trianglesPlanar)));
        //if empty space is cut off, reduce cost by 20%
        const double factor = trianglesMin == 0 || trianglesMax == 0 ? 0.8 : 1;
        if (costLesser <= costUpper) {
            return {factor * costLesser, true};
        }
        //if empty space is cut off, reduce cost by 20%
        return {factor * costUpper, false};
    }

    double KDTree::surfaceAreaOfBox(const Box &box) {
        const double width = std::abs(box.maxPoint[0] - box.minPoint[0]);
        const double length = std::abs(box.maxPoint[1] - box.minPoint[1]);
        const double height = std::abs(box.maxPoint[2] - box.minPoint[2]);
        return 2 * (width * length + width * height + length * height);
    }

}// namespace polyhedralGravity