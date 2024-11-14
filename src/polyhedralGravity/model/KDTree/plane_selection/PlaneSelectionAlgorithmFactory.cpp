#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithmFactory.h"

namespace polyhedralGravity {
    std::shared_ptr<PlaneSelectionAlgorithm> PlaneSelectionAlgorithmFactory::create(const PlaneSelectionAlgorithm::Algorithm algorithm) {
        using Algorithm = PlaneSelectionAlgorithm::Algorithm;
        switch (algorithm) {
            case Algorithm::NOTREE:
                return std::make_shared<NoTreePlane>();
            case Algorithm::QUADRATIC:
                return std::make_shared<SquaredPlane>();
            //TODO: include log algorithm when fixed
            default:
            case Algorithm::LOG:
            case Algorithm::LOGSQUARED:
                return std::make_shared<LogNSquaredPlane>();
                return std::make_shared<LogNPlane>();
        }
    }
}// namespace polyhedralGravity
