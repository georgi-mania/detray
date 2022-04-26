/** Detray library, part of the ACTS project (R&D line)
 *
 * (c) 2022 CERN for the benefit of the ACTS project
 *
 * Mozilla Public License Version 2.0
 */

#pragma once

#if defined(array)
#include "detray/plugins/algebra/array_definitions.hpp"
#elif defined(eigen)
#include "detray/plugins/algebra/eigen_definitions.hpp"
#elif defined(smatrix)
#include "detray/plugins/algebra/smatrix_definitions.hpp"
#elif defined(vc_array)
#include "detray/plugins/algebra/vc_array_definitions.hpp"
#endif

#include "detray/definitions/units.hpp"
#include "detray/field/constant_magnetic_field.hpp"
#include "detray/propagator/rk_stepper.hpp"
#include "detray/propagator/track.hpp"
#include "vecpar/core/algorithms/parallelizable_map.hpp"
#include "vecpar/core/definitions/config.hpp"

using namespace detray;

// type definitions
using vector3 = __plugin::vector3<scalar>;
using point3 = __plugin::point3<scalar>;

using mag_field_type = constant_magnetic_field<>;
using rk_stepper_type = rk_stepper<mag_field_type, free_track_parameters>;

// geometry navigation configurations
constexpr unsigned int theta_steps = 100;
constexpr unsigned int phi_steps = 100;
constexpr unsigned int rk_steps = 100;

constexpr scalar epsilon = 1e-5;
constexpr scalar path_limit = 2 * unit_constants::m;

namespace detray {

// dummy navigation struct
struct nav_state {
    DETRAY_HOST_DEVICE scalar operator()() const {
        return 1. * unit_constants::mm;
    }
    DETRAY_HOST_DEVICE inline void set_full_trust() {}
    DETRAY_HOST_DEVICE inline void set_high_trust() {}
    DETRAY_HOST_DEVICE inline void set_fair_trust() {}
    DETRAY_HOST_DEVICE inline void set_no_trust() {}
    DETRAY_HOST_DEVICE inline bool abort() { return false; }
};

static inline vecpar::config getConfig() {

#if !defined(__CUDA__)
    vecpar::config config{};  // let the OpenMP runtime choose
#else
    constexpr int thread_dim = 2 * 32;
    constexpr int block_dim = theta_steps * phi_steps / thread_dim + 1;
    vecpar::config config{block_dim, thread_dim};
#endif
    return config;
}

struct rk_stepper_algorithm
    : public vecpar::algorithm::parallelizable_mmap<free_track_parameters,
                                                    vector3> {

    TARGET free_track_parameters& map(free_track_parameters& traj,
                                      vector3 B) override {
        // Define RK stepper
        rk_stepper_type rk(B);
        nav_state n_state{};

        // Forward direction
        rk_stepper_type::state forward_state(traj);
        for (unsigned int i_s = 0; i_s < rk_steps; i_s++) {
            rk.step(forward_state, n_state);
        }

        // Backward direction
        traj.flip();
        rk_stepper_type::state backward_state(traj);
        for (unsigned int i_s = 0; i_s < rk_steps; i_s++) {
            rk.step(backward_state, n_state);
        }

        return traj;
    }

};  // end algorithm

}  // namespace detray