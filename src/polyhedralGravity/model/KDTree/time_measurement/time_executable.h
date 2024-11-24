#pragma once

#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithm.h"
#include "polyhedralGravity/model/Polyhedron.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <tuple>
#include <vector>

namespace polyhedralGravity {
    /**
     * Interface in order to store Functions with different argument types into the same vector (type erasure).
     */
    struct IFunction {
        //prevent slicing
        virtual ~IFunction() = default;

        /**
         * Measures the execution time.
         * @return The time in milliseconds.
         */
        [[nodiscard]] virtual size_t measureTimeMs() const = 0;

        /**
         * How the function should be referred to when measurement times are printed later.
         */
        std::string_view name;

        explicit IFunction(const std::string_view name)
            : name(name) {
        }
    };

    /**
     * Struct to bundle all necessary information to perform time measurements for a function.
     * @tparam Args The parameters passed to the function.
     */
    template<typename ReturnType, typename... Args>
    struct Function final : IFunction {
        /**
         * The function to be measured.
         */
        std::function<ReturnType(Args...)> func;

        /**
         * The arguments that are passed to the function.
         */
        std::tuple<Args...> values;

        Function(const std::string_view name, std::function<ReturnType(Args...)> func, Args... values)
            : IFunction(name), func{func}, values{values...} {
        }

        [[nodiscard]] size_t measureTimeMs() const override {
            const auto begin = std::chrono::high_resolution_clock::now();
            std::apply(func, values);
            const auto end = std::chrono::high_resolution_clock::now();
            return std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        }
    };

    /**
     * Measures how long integrity checks on the EROS polyhedron take using different KDTree algorithms.
     */
    void measureTreePerformance();
}// namespace polyhedralGravity
