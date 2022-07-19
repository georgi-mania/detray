#ifndef DETRAY_RK_BOUND_ALG_HPP
#define DETRAY_RK_BOUND_ALG_HPP

#include "common.hpp"

namespace detray {

struct rk_stepper_bound_algorithm
    : public vecpar::algorithm::parallelizable_map<
          vecpar::collection::One, vecmem::vector<bound_track_parameters>,
          vecmem::vector<bound_track_parameters>, const vector3,
          const transform3> {

    TARGET bound_track_parameters& map(bound_track_parameters& out_param,
                                       const bound_track_parameters& in_param,
                                       const vector3& B,
                                       const transform3& trf) override {

        mag_field_t mag_field(B);
        prop_state<crk_stepper_t::state, nav_state> propagation{
            crk_stepper_t::state(in_param, trf), nav_state{}};
        crk_stepper_t::state& crk_state = propagation._stepping;
        nav_state& n_state = propagation._navigation;

        // Decrease tolerance down to 1e-8
        crk_state.set_tolerance(rk_tolerance);

        // RK stepper and its state
        crk_stepper_t crk_stepper(mag_field);

        // Path length per turn
        scalar S = 2. * std::fabs(1. / in_param.qop()) / getter::norm(B) * M_PI;

        // Run stepper for half turn
        unsigned int max_steps = 1e4;

        for (unsigned int i = 0; i < max_steps; i++) {

            crk_state.set_constraint(S - crk_state.path_length());

            n_state._step_size = S;

            crk_stepper.step(propagation);

            if (std::abs(S - crk_state.path_length()) < 1e-6) {
                break;
            }
        }

        // Bound state after one turn propagation
        out_param = crk_stepper.bound_state(propagation, trf);
        return out_param;
    }

};  // end algorithm

}  // namespace detray
#endif