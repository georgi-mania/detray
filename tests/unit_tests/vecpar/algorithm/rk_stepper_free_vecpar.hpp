#ifndef DETRAY_RK_FREE_ALG_HPP
#define DETRAY_RK_FREE_ALG_HPP

#include "common.hpp"

namespace detray {

struct rk_stepper_free_algorithm
    : public vecpar::algorithm::parallelizable_mmap<
          vecpar::collection::One, vecmem::vector<free_track_parameters>,
          const vector3> {

    TARGET free_track_parameters& map(free_track_parameters& track,
                                      const vector3& B) override {

        free_track_parameters c_traj(track);

        // Define RK stepper
        rk_stepper_t rk_stepper(B);
        crk_stepper_t crk_stepper(B);

        // RK Stepping into forward direction
        prop_state<rk_stepper_t::state, nav_state> propagation{
            rk_stepper_t::state{track}, nav_state{}};
        prop_state<crk_stepper_t::state, nav_state> c_propagation{
            crk_stepper_t::state{c_traj}, nav_state{}};

        crk_stepper_t::state& crk_state = c_propagation._stepping;

        nav_state& n_state = propagation._navigation;
        nav_state& cn_state = c_propagation._navigation;

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
        n_state._step_size *= -1. * unit_constants::mm;
        cn_state._step_size *= -1. * unit_constants::mm;

        for (unsigned int i_s = 0; i_s < rk_steps; i_s++) {
            rk_stepper.step(propagation);
            crk_stepper.step(c_propagation);
            crk_stepper.step(c_propagation);
        }

        return track;
    }
};  // end algorithm

}  // namespace detray

#endif