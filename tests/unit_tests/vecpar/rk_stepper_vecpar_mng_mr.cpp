/** Detray library, part of the ACTS project (R&D line)
 *
 * (c) 2022 CERN for the benefit of the ACTS project
 *
 * Mozilla Public License Version 2.0
 */

#include <gtest/gtest.h>

#include "algorithm/rk_stepper_bound_vecpar.hpp"
#include "algorithm/rk_stepper_free_vecpar.hpp"
#include "vecmem/memory/cuda/managed_memory_resource.hpp"
#include "vecmem/memory/host_memory_resource.hpp"
#include "vecpar/all/main.hpp"

/*
TEST(rk_stepper_vecpar, free_state_mng_mr) {

    std::cout << "[rk_stepper_vecpar] free_state managed memory" << std::endl;

    // VecMem memory resource(s)
    vecmem::host_memory_resource host_mr;
    vecmem::cuda::managed_memory_resource mng_mr;

    // Create the vector of initial track parameters
    vecmem::vector<free_track_parameters> tracks_host(&mng_mr);
    vecmem::vector<free_track_parameters> tracks_device(&mng_mr);

    // Create the vector of accumulated path lengths
    vecmem::vector<scalar> path_lengths(&host_mr);

    // Set origin position of tracks
    const point3 ori{0., 0., 0.};

    // Set the magnetic field
    const vector3 B{0, 0, 2 * unit_constants::T};

    // Define RK stepper
    rk_stepper_t rk_stepper(B);
    crk_stepper_t crk_stepper(B);

    // Loops of theta values ]0,pi[
    for (unsigned int itheta = 0; itheta < theta_steps; ++itheta) {
        scalar theta = 0.001 + itheta * (M_PI - 0.001) / theta_steps;
        scalar sin_theta = std::sin(theta);
        scalar cos_theta = std::cos(theta);

        // Loops of phi values [-pi, pi]
        for (unsigned int iphi = 0; iphi < phi_steps; ++iphi) {
            // The direction
            scalar phi = -M_PI + iphi * (2 * M_PI) / phi_steps;
            scalar sin_phi = std::sin(phi);
            scalar cos_phi = std::cos(phi);
            const vector3 dir{cos_phi * sin_theta, sin_phi * sin_theta,
                              cos_theta};

            // intialize a track
            free_track_parameters traj(ori, 0, dir, -1);

            tracks_host.push_back(traj);
            tracks_device.push_back(traj);
        }
    }

    for (unsigned int i = 0; i < theta_steps * phi_steps; i++) {

        auto &traj = tracks_host[i];
        free_track_parameters c_traj(traj);

        // RK Stepping into forward direction
        prop_state<rk_stepper_t::state, nav_state> propagation{
            rk_stepper_t::state{traj}, nav_state{}};
        prop_state<crk_stepper_t::state, nav_state> c_propagation{
            crk_stepper_t::state{c_traj}, nav_state{}};

        rk_stepper_t::state &rk_state = propagation._stepping;
        crk_stepper_t::state &crk_state = c_propagation._stepping;

        nav_state &n_state = propagation._navigation;
        nav_state &cn_state = c_propagation._navigation;

        crk_state.template set_constraint<step::constraint::e_user>(
            0.5 * unit_constants::mm);
        n_state._step_size = 1. * unit_constants::mm;
        cn_state._step_size = 1. * unit_constants::mm;
        ASSERT_NEAR(crk_state.constraints().template size<>(),
                    0.5 * unit_constants::mm, epsilon);
        for (unsigned int i_s = 0; i_s < rk_steps; i_s++) {
            rk_stepper.step(propagation);
            crk_stepper.step(c_propagation);
            crk_stepper.step(c_propagation);
        }

        // check constrained steps
        EXPECT_NEAR(rk_state.path_length(), crk_state.path_length(), epsilon);

        // Backward direction
        // Roll the same track back to the origin
        // Use the same path length, since there is no overstepping
        scalar path_length = rk_state.path_length();
        n_state._step_size *= -1. * unit_constants::mm;
        cn_state._step_size *= -1. * unit_constants::mm;
        for (unsigned int i_s = 0; i_s < rk_steps; i_s++) {
            rk_stepper.step(propagation);
            crk_stepper.step(c_propagation);
            crk_stepper.step(c_propagation);
        }

        EXPECT_NEAR(rk_state.path_length(), crk_state.path_length(), epsilon);

        path_lengths.push_back(2 * path_length);
    }

    // Run RK stepper vecpar
    rk_stepper_free_algorithm rk_stepper_algo;
    vecpar::parallel_map(rk_stepper_algo, mng_mr, vecpar_config(),
tracks_device, B);

    for (unsigned int i = 0; i < theta_steps * phi_steps; i++) {
        auto host_pos = tracks_host[i].pos();
        auto device_pos = tracks_device[i].pos();

        auto host_relative_error = 1. / path_lengths[i] * (host_pos - ori);
        auto device_relative_error = 1. / path_lengths[i] * (device_pos - ori);

        EXPECT_NEAR(getter::norm(host_relative_error), 0, epsilon);
        EXPECT_NEAR(getter::norm(device_relative_error), 0, epsilon);
    }
}

TEST(rk_stepper_vecpar, bound_state_mng_mr) {

    std::cout << "[rk_stepper_vecpar] bound_state managed memory" << std::endl;

    // VecMem memory resource(s)
    vecmem::cuda::managed_memory_resource mng_mr;

    // test surface
    const vector3 u{0, 1, 0};
    const vector3 w{1, 0, 0};
    const vector3 t{0, 0, 0};
    const transform3 trf(t, w, u);

    // Generate track starting point
    vector3 local{2, 3, 0};
    vector3 mom{0.02, 0., 0.};
    scalar time = 0.;
    scalar q = -1.;

    vecmem::vector<bound_track_parameters> in_params(&mng_mr);
            // bound vector
            typename bound_track_parameters::vector_type bound_vector;
            getter::element(bound_vector, e_bound_loc0, 0) = local[0];
            getter::element(bound_vector, e_bound_loc1, 0) = local[1];
            getter::element(bound_vector, e_bound_phi, 0) = getter::phi(mom);
            getter::element(bound_vector, e_bound_theta, 0) =
getter::theta(mom); getter::element(bound_vector, e_bound_qoverp, 0) = q /
getter::norm(mom); getter::element(bound_vector, e_bound_time, 0) = time;

            // bound covariance
            typename bound_track_parameters::covariance_type bound_cov =
                matrix_operator().template zero<e_bound_size, e_bound_size>();
            getter::element(bound_cov, e_bound_loc0, e_bound_loc0) = 1.;
            getter::element(bound_cov, e_bound_loc1, e_bound_loc1) = 1.;
            getter::element(bound_cov, e_bound_phi, e_bound_phi) = 1.;

            // Note: Set theta error as ZERO, to constrain the loc1 divergence
            getter::element(bound_cov, e_bound_theta, e_bound_theta) = 0.;
            getter::element(bound_cov, e_bound_qoverp, e_bound_qoverp) = 1.;

            // bound track parameter
            const bound_track_parameters in_param(0, bound_vector, bound_cov);
            in_params.push_back(in_param);

    const vector3 B{0, 0, 1. * unit_constants::T};
    mag_field_t mag_field(B);

    /// Get CPU bound parameter after one turn

    vecmem::vector<bound_track_parameters> out_param_cpu(&mng_mr);
    for (unsigned int i = 0; i < in_params.size(); i++) {

        auto& traj = in_params[i];

        prop_state<crk_stepper_t::state, nav_state> propagation{
            crk_stepper_t::state(traj, trf), nav_state{}};
        crk_stepper_t::state &crk_state = propagation._stepping;
        nav_state &n_state = propagation._navigation;

        // Decrease tolerance down to 1e-8
        crk_state.set_tolerance(rk_tolerance);

        // RK stepper and its state
        crk_stepper_t crk_stepper(mag_field);

        // Path length per turn
        scalar S = 2. * std::fabs(1. / traj.qop()) / getter::norm(B) * M_PI;

        // Run stepper for half turn
        unsigned int max_steps = 1e4;

        for (unsigned int j = 0; j < max_steps; j++) {

            crk_state.set_constraint(S - crk_state.path_length());

            n_state._step_size = S;

            crk_stepper.step(propagation);

            if (std::abs(S - crk_state.path_length()) < 1e-6) {
                break;
            }

            // Make sure that we didn't reach the end of for loop
            ASSERT_TRUE(j < max_steps - 1);
        }
        // Bound state after one turn propagation
        const auto out_param = crk_stepper.bound_state(propagation, trf);
        out_param_cpu.push_back(out_param);
    }

     /// Get vecpar bound parameter after one turn

    // Run RK stepper
    rk_stepper_bound_algorithm rk_stepper_algo;
    vecmem::vector<bound_track_parameters> out_param_gpu =
        vecpar::parallel_map(rk_stepper_algo, mng_mr, vecpar::config{1, 1},
in_params, B, trf);

    /// Compare CPU and vecpar CPU/GPU

    for (unsigned int k = 0; k < in_params.size(); k++) {
        const auto bvec_cpu = out_param_cpu[k].vector();
        const auto bcov_cpu = out_param_cpu[k].covariance();

        const auto bvec_gpu = out_param_gpu[k].vector();
        const auto bcov_gpu = out_param_gpu[k].covariance();

        for (size_type i = 0; i < e_bound_size; i++) {
            EXPECT_NEAR(matrix_operator().element(bvec_cpu, i, 0),
                        matrix_operator().element(bvec_gpu, i, 0), epsilon);
        }

        for (size_type i = 0; i < e_bound_size; i++) {
            for (size_type j = 0; j < e_bound_size; j++) {
                EXPECT_NEAR(matrix_operator().element(bcov_cpu, i, j),
                            matrix_operator().element(bcov_gpu, i, j),
                            epsilon);
            }
        }
    }
}

*/

#include "TimeLogger.hpp"

TEST(rk_stepper_vecpar, free_state_mng_mr) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
    std::cout << "[rk_stepper_vecpar] free_state managed memory" << std::endl;

    // VecMem memory resource(s)
    vecmem::host_memory_resource host_mr;
    vecmem::cuda::managed_memory_resource mng_mr;

    // Create the vector of initial track parameters
    vecmem::vector<free_track_parameters> tracks_host(&mng_mr);
    vecmem::vector<free_track_parameters> tracks_device(&mng_mr);

    // Create the vector of accumulated path lengths
    vecmem::vector<scalar> path_lengths(&host_mr);

    // Set origin position of tracks
    const point3 ori{0., 0., 0.};

    // Set the magnetic field
    const vector3 B{0, 0, 2 * unit_constants::T};

    // Define RK stepper
    rk_stepper_t rk_stepper(B);
    crk_stepper_t crk_stepper(B);

    // Loops of theta values ]0,pi[
    for (unsigned int itheta = 0; itheta < theta_steps; ++itheta) {
        scalar theta = 0.001 + itheta * (M_PI - 0.001) / theta_steps;
        scalar sin_theta = std::sin(theta);
        scalar cos_theta = std::cos(theta);

        // Loops of phi values [-pi, pi]
        for (unsigned int iphi = 0; iphi < phi_steps; ++iphi) {
            // The direction
            scalar phi = -M_PI + iphi * (2 * M_PI) / phi_steps;
            scalar sin_phi = std::sin(phi);
            scalar cos_phi = std::cos(phi);
            const vector3 dir{cos_phi * sin_theta, sin_phi * sin_theta,
                              cos_theta};

            // intialize a track
            free_track_parameters traj(ori, 0, dir, -1);

            tracks_host.push_back(traj);
            tracks_device.push_back(traj);
        }
    }

    start_time = std::chrono::high_resolution_clock::now();
#if defined(_OPENMP) && !defined(__CUDA__)
#pragma omp parallel for
#endif
    for (unsigned int i = 0; i < theta_steps * phi_steps; i++) {

        auto &traj = tracks_host[i];
        free_track_parameters c_traj(traj);

        // RK Stepping into forward direction
        prop_state<rk_stepper_t::state, nav_state> propagation{
            rk_stepper_t::state{traj}, nav_state{}};
        prop_state<crk_stepper_t::state, nav_state> c_propagation{
            crk_stepper_t::state{c_traj}, nav_state{}};

        rk_stepper_t::state &rk_state = propagation._stepping;
        crk_stepper_t::state &crk_state = c_propagation._stepping;

        nav_state &n_state = propagation._navigation;
        nav_state &cn_state = c_propagation._navigation;

        crk_state.template set_constraint<step::constraint::e_user>(
            0.5 * unit_constants::mm);
        n_state._step_size = 1. * unit_constants::mm;
        cn_state._step_size = 1. * unit_constants::mm;

        for (unsigned int i_s = 0; i_s < rk_steps; i_s++) {
            rk_stepper.step(propagation);
            crk_stepper.step(c_propagation);
            crk_stepper.step(c_propagation);
        }

        // Backward direction
        // Roll the same track back to the origin
        // Use the same path length, since there is no overstepping
        scalar path_length = rk_state.path_length();
        n_state._step_size *= -1. * unit_constants::mm;
        cn_state._step_size *= -1. * unit_constants::mm;
        for (unsigned int i_s = 0; i_s < rk_steps; i_s++) {
            rk_stepper.step(propagation);
            crk_stepper.step(c_propagation);
            crk_stepper.step(c_propagation);
        }
    }

    end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> benchmark_time = end_time - start_time;

#if defined(__CUDA__)
    printf("Sequential time  = %f s\n", benchmark_time.count());
    write_to_csv("rk_stepper_free_mng_cpu_seq.csv", benchmark_time.count());
#elif defined(_OPENMP)
    printf("OMP time  = %f s\n", benchmark_time.count());
    write_to_csv("rk_stepper_free_mng_cpu_omp.csv", benchmark_time.count());
#endif

    start_time = std::chrono::high_resolution_clock::now();

    // Run RK stepper vecpar
    rk_stepper_free_algorithm rk_stepper_algo;
    vecpar::parallel_map(rk_stepper_algo, mng_mr, vecpar_config(),
                         tracks_device, B);

    end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> vecpar_time = end_time - start_time;

#if defined(__CUDA__)
    printf("vecpar gpu time  = %f s\n", vecpar_time.count());
    write_to_csv("rk_stepper_free_mng_gpu_vecpar.csv", vecpar_time.count());
#elif defined(_OPENMP)
    printf("vecpar cpu time  = %f s\n", vecpar_time.count());
    write_to_csv("rk_stepper_free_mng_cpu_vecpar.csv", vecpar_time.count());
#endif
}

TEST(rk_stepper_vecpar, bound_state_mng_mr) {

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
    std::cout << "[rk_stepper_vecpar] bound_state managed memory" << std::endl;

    // VecMem memory resource(s)
    vecmem::cuda::managed_memory_resource mng_mr;

    // test surface
    const vector3 u{0, 1, 0};
    const vector3 w{1, 0, 0};
    const vector3 t{0, 0, 0};
    const transform3 trf(t, w, u);

    // Generate track starting point
    vector3 local{2, 3, 0};
    vector3 mom{0.02, 0., 0.};
    scalar time = 0.;
    scalar q = -1.;

    vecmem::vector<bound_track_parameters> in_params(&mng_mr);
    // bound vector
    typename bound_track_parameters::vector_type bound_vector;
    getter::element(bound_vector, e_bound_loc0, 0) = local[0];
    getter::element(bound_vector, e_bound_loc1, 0) = local[1];
    getter::element(bound_vector, e_bound_phi, 0) = getter::phi(mom);
    getter::element(bound_vector, e_bound_theta, 0) = getter::theta(mom);
    getter::element(bound_vector, e_bound_qoverp, 0) = q / getter::norm(mom);
    getter::element(bound_vector, e_bound_time, 0) = time;

    // bound covariance
    typename bound_track_parameters::covariance_type bound_cov =
        matrix_operator().template zero<e_bound_size, e_bound_size>();
    getter::element(bound_cov, e_bound_loc0, e_bound_loc0) = 1.;
    getter::element(bound_cov, e_bound_loc1, e_bound_loc1) = 1.;
    getter::element(bound_cov, e_bound_phi, e_bound_phi) = 1.;

    // Note: Set theta error as ZERO, to constrain the loc1 divergence
    getter::element(bound_cov, e_bound_theta, e_bound_theta) = 0.;
    getter::element(bound_cov, e_bound_qoverp, e_bound_qoverp) = 1.;

    // bound track parameter
    const bound_track_parameters in_param(0, bound_vector, bound_cov);
    in_params.push_back(in_param);

    const vector3 B{0, 0, 1. * unit_constants::T};
    mag_field_t mag_field(B);

    /// Get CPU bound parameter after one turn

    vecmem::vector<bound_track_parameters> out_param_cpu(&mng_mr);
    for (unsigned int i = 0; i < in_params.size(); i++) {

        auto &traj = in_params[i];

        prop_state<crk_stepper_t::state, nav_state> propagation{
            crk_stepper_t::state(traj, trf), nav_state{}};
        crk_stepper_t::state &crk_state = propagation._stepping;
        nav_state &n_state = propagation._navigation;

        // Decrease tolerance down to 1e-8
        crk_state.set_tolerance(rk_tolerance);

        // RK stepper and its state
        crk_stepper_t crk_stepper(mag_field);

        // Path length per turn
        scalar S = 2. * std::fabs(1. / traj.qop()) / getter::norm(B) * M_PI;

        // Run stepper for half turn
        unsigned int max_steps = 1e4;

        for (unsigned int j = 0; j < max_steps; j++) {

            crk_state.set_constraint(S - crk_state.path_length());

            n_state._step_size = S;

            crk_stepper.step(propagation);

            if (std::abs(S - crk_state.path_length()) < 1e-6) {
                break;
            }

            // Make sure that we didn't reach the end of for loop
            ASSERT_TRUE(j < max_steps - 1);
        }
        // Bound state after one turn propagation
        const auto out_param = crk_stepper.bound_state(propagation, trf);
        out_param_cpu.push_back(out_param);
    }

    /// Get vecpar bound parameter after one turn

    start_time = std::chrono::high_resolution_clock::now();
    // Run RK stepper
    rk_stepper_bound_algorithm rk_stepper_algo;
    vecmem::vector<bound_track_parameters> out_param_gpu = vecpar::parallel_map(
        rk_stepper_algo, mng_mr, vecpar::config{1, 1}, in_params, B, trf);

    end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> vecpar_time = end_time - start_time;

#if defined(__CUDA__)
    printf("vecpar gpu time  = %f s\n", vecpar_time.count());
    write_to_csv("rk_stepper_bound_mng_gpu_vecpar.csv", vecpar_time.count());
#elif defined(_OPENMP)
    printf("vecpar cpu time  = %f s\n", vecpar_time.count());
    write_to_csv("rk_stepper_bound_mng_cpu_vecpar.csv", vecpar_time.count());
#endif
}
