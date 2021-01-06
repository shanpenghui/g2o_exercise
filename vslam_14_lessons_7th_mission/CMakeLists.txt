cmake_minimum_required(VERSION 2.8)

project(7-2)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

#LIST( APPEND CMAKE_MODULE_PATH /home/sph/Documents/slam/g2o_exercise/g2o/cmake_modules )
#SET( G2O_ROOT /usr/local/include/g2o )
#find_package(G2O REQUIRED)
#IF(G2O_FOUND)
#    include_directories(${G2O_INCLUDE_DIR})
#    message("G2O lib is found:" ${G2O_INCLUDE_DIR})
#ENDIF(G2O_FOUND)

set(CMAKE_MODULE_PATH "/home/sph/Documents/slam/g2o_exercise/g2o/cmake_modules;${CMAKE_MODULE_PATH}")
find_package(G2O REQUIRED)

option(G2O_USE_CHOLMOD "Build g2o with CHOLMOD support" ON)
#set( PATH "/usr/lib/x86_64-linux-gnu/:${PATH}")
find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)
if (G2O_USE_CHOLMOD AND CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
    message(STATUS "Enable support for Cholmod")
    set(CHOLMOD_FOUND TRUE)
else()
    message(STATUS "Disable support for Cholmod")
    set(CHOLMOD_FOUND FALSE)
endif()
set(G2O_HAVE_CHOLMOD ${CHOLMOD_FOUND})

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
#        /usr/include/suitesparse
)

find_package(SuiteSparse REQUIRED)
include_directories(${SUITESPARSE_INCLUDE_DIRS})

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator
        g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
        g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
        g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper
        g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg
        g2o_types_data g2o_types_sim3 g2o_csparse_extension
        )

add_executable(BA_large main.cpp data.cpp BundleAdjustment.cpp)

if(CHOLMOD_FOUND)
    target_link_libraries(BA_large ${OpenCV_LIBS} ${G2O_LIBS} ${SuiteSparse_LIBRARIES})
else()
    target_link_libraries(BA_large ${OpenCV_LIBS} ${G2O_LIBS} solver_eigen)
endif()
