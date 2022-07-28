# Detray library, part of the ACTS project (R&D line)
#
# (c) 2022 CERN for the benefit of the ACTS project
#
# Mozilla Public License Version 2.0


# make unit tests for multiple algebras
set(algebras "array")
if (DETRAY_EIGEN_PLUGIN)
    list(APPEND algebras "eigen")
endif ()

## -fno-fast-math -ffast-math

### CPU tests
find_package(OpenMP QUIET)
if ((TARGET vecpar::all) AND (TARGET vecpar::omp))
    foreach (algebra ${algebras})
        if (DETRAY_BUILD_CUDA)
            find_package(CUDAToolkit)
            # if CUDA libraries available,
            # run tests with host and managed memory
            detray_add_test(${algebra}_vecpar_cpu
                    "rk_stepper_vecpar_mng_mr.cpp"
                    "rk_stepper_vecpar_host_mr.cpp"
                    "algorithm/rk_stepper_bound_vecpar.hpp"
                    "algorithm/rk_stepper_free_vecpar.hpp"
                    "algorithm/common.hpp"
                    LINK_LIBRARIES GTest::gtest_main vecmem::cuda detray_tests_common
                    detray::${algebra} vecpar::all CUDA::cudart OpenMP::OpenMP_CXX)
        else ()
            # if CUDA libraries not available,
            # run tests with host memory only
            detray_add_test(${algebra}_vecpar_cpu
                    "rk_stepper_vecpar_host_mr.cpp"
                    "algorithm/rk_stepper_bound_vecpar.hpp"
                    "algorithm/rk_stepper_free_vecpar.hpp"
                    "algorithm/common.hpp"
                    LINK_LIBRARIES GTest::gtest_main vecmem::core detray_tests_common
                    detray::${algebra} vecpar::all OpenMP::OpenMP_CXX)
        endif ()

        target_compile_definitions(detray_test_${algebra}_vecpar_cpu
                PRIVATE ${algebra}=${algebra})

        set_target_properties(detray_test_${algebra}_vecpar_cpu
                PROPERTIES CXX_STANDARD 20)

        set_target_properties(detray_test_${algebra}_vecpar_cpu
                PROPERTIES LINKER_LANGUAGE CXX)

    endforeach ()
endif ()

### GPU tests can be built only by Clang compiler +nvptx support
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if (DETRAY_BUILD_CUDA)
        if ((TARGET vecpar::all) AND (TARGET vecpar::cuda))
            foreach (algebra ${algebras})

                # Unit tests for the selected algebra.
                detray_add_test(${algebra}_vecpar_gpu
                        "rk_stepper_vecpar_mng_mr.cpp"
                        "rk_stepper_vecpar_host_mr.cpp"
                        "algorithm/rk_stepper_bound_vecpar.hpp"
                        "algorithm/rk_stepper_free_vecpar.hpp"
                        "algorithm/common.hpp"
                        LINK_LIBRARIES GTest::gtest_main vecmem::cuda detray_tests_common
                        detray::${algebra} vecpar::all CUDA::cudart)

                target_compile_definitions(detray_test_${algebra}_vecpar_gpu
                        PRIVATE ${algebra}=${algebra})

                set_target_properties(detray_test_${algebra}_vecpar_gpu
                        PROPERTIES CXX_STANDARD 20)

                set_target_properties(detray_test_${algebra}_vecpar_gpu
                        PROPERTIES LINKER_LANGUAGE CXX)

                target_compile_options(detray_test_${algebra}_vecpar_gpu PUBLIC
                        $<$<COMPILE_LANGUAGE:CXX>:-x cuda --offload-arch=sm_70 -fno-fast-math>)
            endforeach ()
        endif ()
    endif ()
endif ()