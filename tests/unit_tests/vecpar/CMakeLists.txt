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
            detray_add_test(vecpar_${algebra}_cpu
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
            detray_add_test(vecpar_${algebra}_cpu
                    "rk_stepper_vecpar_host_mr.cpp"
                    "algorithm/rk_stepper_bound_vecpar.hpp"
                    "algorithm/rk_stepper_free_vecpar.hpp"
                    "algorithm/common.hpp"
                    LINK_LIBRARIES GTest::gtest_main vecmem::core detray_tests_common
                    detray::${algebra} vecpar::all OpenMP::OpenMP_CXX)
        endif ()

        target_compile_definitions(detray_test_vecpar_${algebra}_cpu
                PRIVATE ${algebra}=${algebra})

        set_target_properties(detray_test_vecpar_${algebra}_cpu
                PROPERTIES CXX_STANDARD 20)

        set_target_properties(detray_test_vecpar_${algebra}_cpu
                PROPERTIES LINKER_LANGUAGE CXX)
    endforeach ()
endif ()

### GPU tests can be built only by Clang compiler +nvptx support
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if (DETRAY_BUILD_CUDA)
        if ((TARGET vecpar::all) AND (TARGET vecpar::cuda))
            foreach (algebra ${algebras})

                # Unit tests for the selected algebra.
                detray_add_test(vecpar_${algebra}_gpu
                        "rk_stepper_vecpar_mng_mr.cpp"
                        "rk_stepper_vecpar_host_mr.cpp"
                        "algorithm/rk_stepper_bound_vecpar.hpp"
                        "algorithm/rk_stepper_free_vecpar.hpp"
                        "algorithm/common.hpp"
                        LINK_LIBRARIES GTest::gtest_main vecmem::cuda detray_tests_common
                        detray::${algebra} vecpar::all CUDA::cudart)

                target_compile_definitions(detray_test_vecpar_${algebra}_gpu
                        PRIVATE ${algebra}=${algebra})

                set_target_properties(detray_test_vecpar_${algebra}_gpu
                        PROPERTIES CXX_STANDARD 20)

                set_target_properties(detray_test_vecpar_${algebra}_gpu
                        PROPERTIES LINKER_LANGUAGE CXX)

                target_compile_options(detray_test_vecpar_${algebra}_gpu PRIVATE
                        $<$<COMPILE_LANGUAGE:CXX>:-x cuda --offload-arch=sm_86 -fno-fast-math -ffp-contract=off>)
            endforeach ()
        endif ()
    endif ()
endif ()


### OpenMP target for CPU
if ((TARGET vecpar::all) AND (TARGET vecpar::omp))
    foreach (algebra ${algebras})
        detray_add_test(vecpar_${algebra}_ompt_cpu
                "rk_stepper_vecpar_ompt_host_mr.cpp"
                "algorithm/rk_stepper_bound_vecpar.hpp"
                "algorithm/rk_stepper_free_vecpar.hpp"
                "algorithm/common.hpp"
                LINK_LIBRARIES GTest::gtest_main vecmem::core detray_tests_common
                detray::${algebra} vecpar::all OpenMP::OpenMP_CXX)

        target_compile_definitions(detray_test_vecpar_${algebra}_ompt_cpu
                PRIVATE ${algebra}=${algebra})

        set_target_properties(detray_test_vecpar_${algebra}_ompt_cpu
                PROPERTIES CXX_STANDARD 20)

        set_target_properties(detray_test_vecpar_${algebra}_ompt_cpu
               PROPERTIES LINKER_LANGUAGE CXX)
      #  detray_add_flag( CMAKE_CXX_FLAGS "-foffload=disable" )
       # target_compile_options(detray_test_vecpar_${algebra}_ompt_cpu PRIVATE -foffload=disable)
        target_compile_options(detray_test_vecpar_${algebra}_ompt_cpu PRIVATE -fopenmp )#-foffload-lto -gline-tables-only)
        target_link_options(detray_test_vecpar_${algebra}_ompt_cpu PRIVATE -fopenmp -fopenmp-targets=nvptx64-nvidia-cuda)
    endforeach ()
endif ()

set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -v")

### OpenMP target for GPU (nvidia)
if ((TARGET vecpar::all) AND (TARGET vecpar::omp))
    foreach (algebra ${algebras})
        detray_add_test(vecpar_${algebra}_ompt_gpu
                "rk_stepper_vecpar_ompt_host_mr.cpp"
                "algorithm/rk_stepper_bound_vecpar.hpp"
                "algorithm/rk_stepper_free_vecpar.hpp"
                "algorithm/common.hpp"
                LINK_LIBRARIES GTest::gtest_main vecmem::core detray_tests_common
                detray::${algebra} vecpar::all OpenMP::OpenMP_CXX )

        target_compile_definitions(detray_test_vecpar_${algebra}_ompt_gpu
                PRIVATE ${algebra}=${algebra})

        set_target_properties(detray_test_vecpar_${algebra}_ompt_gpu
                PROPERTIES CXX_STANDARD 20)

        set_target_properties(detray_test_vecpar_${algebra}_ompt_gpu
                PROPERTIES LINKER_LANGUAGE CXX)
        #detray_add_flag( CMAKE_CXX_FLAGS "-foffload=nvptx-none" )
        target_compile_options(detray_test_vecpar_${algebra}_ompt_gpu PRIVATE -fopenmp -fopenmp-targets=nvptx64-nvidia-cuda --offload-arch=sm_86 -gline-tables-only ) #-O3 -foffload-lto
        target_link_options(detray_test_vecpar_${algebra}_ompt_gpu PRIVATE -fopenmp -fopenmp-targets=nvptx64-nvidia-cuda ) #-foffload-lto -O3
        target_compile_definitions(detray_test_vecpar_${algebra}_ompt_gpu PRIVATE COMPILE_FOR_DEVICE)
    endforeach ()
endif ()
