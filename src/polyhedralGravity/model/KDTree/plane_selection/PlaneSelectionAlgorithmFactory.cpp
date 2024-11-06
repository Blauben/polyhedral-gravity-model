#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithmFactory.h"

namespace polyhedralGravity {
    std::unique_ptr<PlaneSelectionAlgorithm> PlaneSelectionAlgorithmFactory::create(const PlaneSelectionAlgorithm::Algorithm algorithm) {
        using Algorithm = PlaneSelectionAlgorithm::Algorithm;
        switch (algorithm) {
            case Algorithm::NOTREE:
                return std::make_unique<NoTreePlane>();
            case Algorithm::QUADRATIC:
                return std::make_unique<SquaredPlane>();
            case Algorithm::LOGSQUARED:
                return std::make_unique<LogNSquaredPlane>();
            default:
            case Algorithm::LOG:
                return std::make_unique<LogNPlane>();
        }
    }
}// namespace polyhedralGravity
