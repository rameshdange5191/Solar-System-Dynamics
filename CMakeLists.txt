cmake_minimum_required(VERSION 3.0)

project(ccp_ssd)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

find_package(pybind11 REQUIRED)

pybind11_add_module(PhyVec_class PhyVec_class.cpp binder.cpp)

# target_link_libraries(trial PRIVATE pybind11::module)
target_link_libraries(PhyVec_class PRIVATE pybind11::module)