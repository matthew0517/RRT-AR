cmake_minimum_required(VERSION 3.12)
project(example)

# Set the Conda environment path
# set(CONDA_ENV_PATH "/home/matthew/anaconda3/envs/RRT-AR")

# Set the CMAKE_PREFIX_PATH to search for packages in the Conda environment
# list(APPEND CMAKE_PREFIX_PATH ${CONDA_ENV_PATH})

add_subdirectory(pybind11)
#set (PYBIND11_PYTHON_VERSION "3.11") 

#find_package(PythonLibs REQUIRED)
#find_package(pybind11 REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -fPIC")

#add_executable(testExample example.cpp)

pybind11_add_module(RRTAR MODULE RRTAR.cpp)



#include_directories(/home/matthew/anaconda3/pkgs/pybind11-2.10.4-py311hdb19cb5_0/lib/python3.11/site-packages/pybind11/include/pybind11)

#add_executable(hello main.cpp)
#add_executable(RRTAR RRTAR.cpp)

#include_directories(${PYTHON_INCLUDE_DIRS})
#target_link_libraries(<your exe or lib> ${PYTHON_LIBRARIES})