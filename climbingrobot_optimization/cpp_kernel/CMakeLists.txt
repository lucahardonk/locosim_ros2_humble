cmake_minimum_required(VERSION 3.14)
project(climbingrobot_kernel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -funroll-loops")

# ---------------------------------------------------------------------------
# Standalone C++ static library (usable directly from a ROS2 C++ node)
# ---------------------------------------------------------------------------
add_library(dynamics_kernel STATIC src/dynamics_kernel.cpp)
target_include_directories(dynamics_kernel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dynamics_kernel PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Optional Python (pybind11) module: climbingrobot_kernel
# ---------------------------------------------------------------------------
option(BUILD_PYTHON_BINDINGS "Build the pybind11 Python module" ON)
if(BUILD_PYTHON_BINDINGS)
  # locate a python interpreter (needed for the pybind11 cmakedir fallback)
  find_package(Python3 COMPONENTS Interpreter Development QUIET)
  if(Python3_EXECUTABLE)
    set(_PY_EXE ${Python3_EXECUTABLE})
  else()
    set(_PY_EXE python3)
  endif()

  find_package(pybind11 CONFIG QUIET)
  if(NOT pybind11_FOUND)
    # fall back to `python -m pybind11 --cmakedir`
    execute_process(
      COMMAND ${_PY_EXE} -m pybind11 --cmakedir
      OUTPUT_VARIABLE PYBIND11_CMAKE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(PYBIND11_CMAKE_DIR)
      list(APPEND CMAKE_PREFIX_PATH ${PYBIND11_CMAKE_DIR})
      find_package(pybind11 CONFIG QUIET)
    endif()
  endif()
  if(pybind11_FOUND)
    pybind11_add_module(climbingrobot_kernel src/bindings.cpp src/dynamics_kernel.cpp)
    target_include_directories(climbingrobot_kernel PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  else()
    message(WARNING "pybind11 not found: skipping Python bindings")
  endif()
endif()

install(TARGETS dynamics_kernel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
