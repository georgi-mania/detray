/** Detray library, part of the ACTS project (R&D line)
 *
 * (c) 2022 CERN for the benefit of the ACTS project
 *
 * Mozilla Public License Version 2.0
 */

#pragma once

#include <cuda.h>

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

#include "detray/definitions/cuda_definitions.hpp"
#include "vecmem/containers/device_vector.hpp"
#include "vecpar/core/algorithms/parallelizable_map.hpp"

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

__global__ void  rk_stepper_test_kernel(
    vecmem::data::vector_view<free_track_parameters> tracks_data,
    const vector3 B) {

    unsigned int gid = threadIdx.x + blockIdx.x * blockDim.x;
    vecmem::device_vector<free_track_parameters> tracks(tracks_data);

    // Prevent overflow
    if (gid >= tracks.size()) {
        return;
    }

    // Define RK stepper
    rk_stepper_type rk(B);
    nav_state n_state{};

    // Get a track
    auto& traj = tracks.at(gid);

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
}

void rk_stepper_test(
        vecmem::data::vector_view<free_track_parameters>& tracks_data,
        const vector3 B) {

    constexpr int thread_dim = 2 * WARP_SIZE;
    constexpr int block_dim = theta_steps * phi_steps / thread_dim + 1;

    // run the test kernel
    rk_stepper_test_kernel<<<block_dim, thread_dim>>>(tracks_data, B);

    // cuda error check
    DETRAY_CUDA_ERROR_CHECK(cudaGetLastError())
    DETRAY_CUDA_ERROR_CHECK(cudaDeviceSynchronize())
}

}  // namespace detray