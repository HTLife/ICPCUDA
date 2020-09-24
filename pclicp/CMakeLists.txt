cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iterative_closest_point)

find_package(PCL 1.2 REQUIRED)


find_package(CUDA REQUIRED)#
find_package(ICPCUDA REQUIRED)#
find_library(CUDART_LIBRARY cudart ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})#
set(CUDA_ARCH_BIN "" CACHE STRING "Specify 'real' GPU arch to build binaries for, BIN(PTX) format is supported. Example: 1.3 2.1(1.3) or 13 21(13)")
set(CUDA_ARCH_PTX "" CACHE STRING "Specify 'virtual' PTX arch to build PTX intermediate code for. Example: 1.0 1.2 or 10 12")              

include("CudaDetect.cmake")
detect_installed_gpus(CUDA_NVCC_ARCHS)
foreach(NVCC_ARCH IN LISTS CUDA_NVCC_ARCHS)
    list(APPEND CUDA_ARCH_BIN "${NVCC_ARCH} ")
endforeach(NVCC_ARCH)

include("CudaComputeTargetFlags.cmake")
APPEND_TARGET_ARCH_FLAGS()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;--expt-relaxed-constexpr")

CUDA_COMPILE(cuda_objs ${cuda})

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3 -Wall")


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (iterative_closest_point iterative_closest_point.cpp)

target_link_libraries (iterative_closest_point ${PCL_LIBRARIES} ${CUDART_LIBRARY} ${CUDA_LIBRARIES} icpcuda)
