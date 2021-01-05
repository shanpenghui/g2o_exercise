cmake_minimum_required(VERSION 2.8)

project(BA_single_camera)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

set(CMAKE_MODULE_PATH "/home/sph/Documents/slam/g2o_exercise/g2o/cmake_modules;${CMAKE_MODULE_PATH}")
find_package(G2O REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
)

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

add_executable(BA_single_camera BA-3Dto2D.cpp)
target_link_libraries(BA_single_camera ${OpenCV_LIBS} ${G2O_LIBS})