#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithmFactory.h"

namespace polyhedralGravity {
    std::shared_ptr<PlaneSelectionAlgorithm> PlaneSelectionAlgorithmFactory::create(const PlaneSelectionAlgorithm::Algorithm algorithm) {
        using Algorithm = PlaneSelectionAlgorithm::Algorithm;
        switch (algorithm) {
            case Algorithm::NOTREE:
                return std::make_unique<NoTreePlane>();
            case Algorithm::QUADRATIC:
                return std::make_unique<SquaredPlane>();
            default:
            case Algorithm::LOG: //TODO
            case Algorithm::LOGSQUARED:
                return std::make_unique<LogNSquaredPlane>();
        }
    }
}// namespace polyhedralGravity
