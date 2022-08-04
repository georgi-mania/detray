#!/bin/sh

spack load llvm@14.0.0
spack load googletest@1.10.0
spack load eigen@3.4.0
spack load cmake@3.22.2
spack load cuda@11.6
echo "Spack environment loaded!"

# the number of repetitions
N=$1

# the number of omp threads
TH=$2

# the destination folder for the results
FILE_F=$4

# make sure the main directory exists
if [ -d "FILE_F" ];
then
    echo "$FILE_F directory already exists"
else
    mkdir $FILE_F
    echo "create $FILE_F directory"
fi 

FILE_ARRAY=$FILE_F/array
if [ -d "$FILE_ARRAY" ];
then
    echo "$FILE_ARRAY directory already exists"
else
    mkdir $FILE_ARRAY
    echo "create $FILE_ARRAY directory"
fi

FILE_EIGEN=$FILE_F/eigen
if [ -d "$FILE_EIGEN" ];
then
    echo "$FILE_EIGEN directory already exists"
else
    mkdir $FILE_EIGEN
    echo "create $FILE_EIGEN directory"
fi

# max threads limit
export OMP_THREAD_LIMIT=($TH+1)
export OMP_NUM_THREADS=$TH

# move to test folder
cd $3

############## CUDA ####################
# check correctness of the result
./detray_test_array_cuda --gtest_filter=rk_stepper_cuda.rk_stepper
./detray_test_array_cuda --gtest_filter=rk_stepper_cuda.bound_state

# performance testing
./detray_test_array_cuda --gtest_repeat=$N --gtest_filter=rk_stepper_cuda.rk_stepper_time
./detray_test_array_cuda --gtest_repeat=$N --gtest_filter=rk_stepper_cuda.bound_state_time
############## END CUDA ####################

############## CPU ####################
# check correctness of the result
./detray_test_array_vecpar_cpu --gtest_filter=rk_stepper_vecpar.free_state_mng_mr
./detray_test_array_vecpar_cpu --gtest_filter=rk_stepper_vecpar.bound_state_mng_mr
./detray_test_array_vecpar_cpu --gtest_filter=rk_stepper_vecpar.free_state_host_mr
./detray_test_array_vecpar_cpu --gtest_filter=rk_stepper_vecpar.bound_state_host_mr

# performance testing
./detray_test_array_vecpar_cpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.free_state_mng_mr_time
./detray_test_array_vecpar_cpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.bound_state_mng_mr_time
./detray_test_array_vecpar_cpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.free_state_host_mr_time
./detray_test_array_vecpar_cpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.bound_state_host_mr_time
############## END CPU ####################

############## GPU ####################
# check correctness of the result
./detray_test_array_vecpar_gpu --gtest_filter=rk_stepper_vecpar.free_state_mng_mr
./detray_test_array_vecpar_gpu --gtest_filter=rk_stepper_vecpar.bound_state_mng_mr
./detray_test_array_vecpar_gpu --gtest_filter=rk_stepper_vecpar.free_state_host_mr
./detray_test_array_vecpar_gpu --gtest_filter=rk_stepper_vecpar.bound_state_host_mr

# performance testing
./detray_test_array_vecpar_gpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.free_state_mng_mr_time
./detray_test_array_vecpar_gpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.bound_state_mng_mr_time
./detray_test_array_vecpar_gpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.free_state_host_mr_time
./detray_test_array_vecpar_gpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.bound_state_host_mr_time
############## END GPU ####################

# move csv to results folder
mv *.csv $FILE_ARRAY
echo "Copy results to $FILE_ARRAY completed!"

############## CUDA ####################
# check correctness of the result
./detray_test_eigen_cuda --gtest_filter=rk_stepper_cuda.rk_stepper
./detray_test_eigen_cuda --gtest_filter=rk_stepper_cuda.bound_state

# performance testing
./detray_test_eigen_cuda --gtest_repeat=$N --gtest_filter=rk_stepper_cuda.rk_stepper_time
./detray_test_eigen_cuda --gtest_repeat=$N --gtest_filter=rk_stepper_cuda.bound_state_time
############## END CUDA ####################

############## CPU ####################
# check correctness of the result
./detray_test_eigen_vecpar_cpu --gtest_filter=rk_stepper_vecpar.free_state_mng_mr
./detray_test_eigen_vecpar_cpu --gtest_filter=rk_stepper_vecpar.bound_state_mng_mr
./detray_test_eigen_vecpar_cpu --gtest_filter=rk_stepper_vecpar.free_state_host_mr
./detray_test_eigen_vecpar_cpu --gtest_filter=rk_stepper_vecpar.bound_state_host_mr

# performance testing
./detray_test_eigen_vecpar_cpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.free_state_mng_mr_time
./detray_test_eigen_vecpar_cpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.bound_state_mng_mr_time
./detray_test_eigen_vecpar_cpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.free_state_host_mr_time
./detray_test_eigen_vecpar_cpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.bound_state_host_mr_time
############## END CPU ####################

############## GPU ####################
# check correctness of the result
./detray_test_eigen_vecpar_gpu --gtest_filter=rk_stepper_vecpar.free_state_mng_mr
./detray_test_eigen_vecpar_gpu --gtest_filter=rk_stepper_vecpar.bound_state_mng_mr
./detray_test_eigen_vecpar_gpu --gtest_filter=rk_stepper_vecpar.free_state_host_mr
./detray_test_eigen_vecpar_gpu --gtest_filter=rk_stepper_vecpar.bound_state_host_mr

# performance testing
./detray_test_eigen_vecpar_gpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.free_state_mng_mr_time
./detray_test_eigen_vecpar_gpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.bound_state_mng_mr_time
./detray_test_eigen_vecpar_gpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.free_state_host_mr_time
./detray_test_eigen_vecpar_gpu --gtest_repeat=$N --gtest_filter=rk_stepper_vecpar.bound_state_host_mr_time
############## END GPU ####################

# move csv to results folder
mv *.csv $FILE_EIGEN
echo "Copy results to $FILE_EIGEN completed!"

# clear environment
spack unload llvm@14.0.0
spack unload googletest@1.10.0
spack unload eigen@3.4.0
spack unload cmake@3.22.2
spack unload cuda@11.6
echo "Spack environment unloaded!"