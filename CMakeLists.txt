cmake_minimum_required(VERSION 3.16)

# vpkg fix
set(VCPKG_TARGET_TRIPLET x64-mingw-static CACHE STRING "")

set(decl_VERSION 1.0.1)

# set the project name and version
project(dcel 
        VERSION ${decl_VERSION}
        DESCRIPTION "Implementation of a doubly connected edge list (DCEL)."
        HOMEPAGE_URL "https://github.com/manisonik/dcel"
        LANGUAGES CXX
        )

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)
find_package(glm CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# create the shared library (DLL)
add_library(dcel dcel-manifold.cpp dcel.cpp dllmain.cpp pch.cpp)
target_link_libraries(dcel PRIVATE glm::glm)
target_link_libraries(dcel PRIVATE Eigen3::Eigen)
target_precompile_headers(dcel PRIVATE pch.h)