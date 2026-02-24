cmake_minimum_required(VERSION 3.8)
project(ecal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ECAL_STANDALONE "Build ecal without ROS/ament" OFF)

find_package(ament_cmake QUIET)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C)

if (ECAL_STANDALONE OR NOT ament_cmake_FOUND)
  set(ECAL_USE_AMENT OFF)
else()
  set(ECAL_USE_AMENT ON)
endif()
# ---------------------------
# Library
# ---------------------------
add_library(ecal
  src/core/iwe_gaussian.cpp
  src/core/cm_velocity_2d.cpp
  src/core/cm_tracker_2d.cpp
  src/core/patch_extractor.cpp
  src/core/corner_init.cpp
  src/core/corner_refinement.cpp
  src/core/checkerboard_validity.cpp
  src/core/checkerboard_calibration.cpp
  src/viz/calibration_viz.cpp
  src/viz/calibration_report.cpp
  src/io/h5_events.cpp
  src/io/calibration_config.cpp
  src/io/calibration_output.cpp
  src/viz/event_render.cpp
)

target_include_directories(ecal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ecal PRIVATE
  ${HDF5_INCLUDE_DIRS}
)

target_link_libraries(ecal
  ${OpenCV_LIBS}
  Ceres::ceres
  ${HDF5_LIBRARIES}
)

# Export include dirs + deps so downstream packages can link cleanly
if (ECAL_USE_AMENT)
  ament_export_include_directories(include)
  ament_export_libraries(ecal)
  ament_target_dependencies(ecal
    OpenCV
    Ceres
  )
endif()

# ---------------------------
# Install
# ---------------------------
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ecal
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if (ECAL_USE_AMENT)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
endif()

# ---------------------------
# Executables
# ---------------------------
function(add_ecal_app target_name source_path)
  add_executable(${target_name}
    ${source_path}
  )

  target_include_directories(${target_name} PRIVATE
    ${HDF5_INCLUDE_DIRS}
  )

  target_link_libraries(${target_name}
    ecal
    ${HDF5_LIBRARIES}
    ${OpenCV_LIBS}
  )

  install(TARGETS ${target_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_ecal_app(ecal_checkerboard_calibrate
  src/app/checkerboard_calibrate.cpp
)

if (ECAL_USE_AMENT)
  ament_package()
endif()
