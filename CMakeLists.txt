cmake_minimum_required(VERSION 3.8)
project(bimbot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)

# Find OpenSCAD for mesh generation
find_program(OPENSCAD_EXECUTABLE openscad)

# Function to convert SCAD to STL
function(generate_stl input_scad output_stl)
    add_custom_command(
        OUTPUT ${output_stl}
        COMMAND ${OPENSCAD_EXECUTABLE} -o ${output_stl} ${input_scad}
        DEPENDS ${input_scad}
        COMMENT "Generating STL from ${input_scad}"
    )
endfunction()

# Mesh Generation
if(OPENSCAD_EXECUTABLE)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/models/builderbot/meshes)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/models/inspectorbot/meshes)

    # Generate BuilderBot meshes
    generate_stl(
        "${CMAKE_CURRENT_SOURCE_DIR}/models/builderbot/meshes/base_link.scad"
        "${CMAKE_CURRENT_BINARY_DIR}/models/builderbot/meshes/base_link.stl"
    )
    generate_stl(
        "${CMAKE_CURRENT_SOURCE_DIR}/models/builderbot/meshes/track.scad"
        "${CMAKE_CURRENT_BINARY_DIR}/models/builderbot/meshes/left_track.stl"
    )
    generate_stl(
        "${CMAKE_CURRENT_SOURCE_DIR}/models/builderbot/meshes/track.scad"
        "${CMAKE_CURRENT_BINARY_DIR}/models/builderbot/meshes/right_track.stl"
    )

    set(BUILDERBOT_STL_FILES
        "${CMAKE_CURRENT_BINARY_DIR}/models/builderbot/meshes/base_link.stl"
        "${CMAKE_CURRENT_BINARY_DIR}/models/builderbot/meshes/left_track.stl"
        "${CMAKE_CURRENT_BINARY_DIR}/models/builderbot/meshes/right_track.stl"
    )

    # Add mesh generation target
    add_custom_target(builderbot_meshes ALL DEPENDS ${BUILDERBOT_STL_FILES})

    # Install generated mesh files
    install(
        FILES ${BUILDERBOT_STL_FILES}
        DESTINATION share/${PROJECT_NAME}/models/builderbot/meshes
    )

    # InspectorBot Mesh Generation
    generate_stl(
        "${CMAKE_CURRENT_SOURCE_DIR}/models/inspectorbot/meshes/base_link.scad"
        "${CMAKE_CURRENT_BINARY_DIR}/models/inspectorbot/meshes/base_link.stl"
    )
    generate_stl(
        "${CMAKE_CURRENT_SOURCE_DIR}/models/inspectorbot/meshes/propeller.scad"
        "${CMAKE_CURRENT_BINARY_DIR}/models/inspectorbot/meshes/propeller.stl"
    )

    # Inspectorbot
    set(INSPECTORBOT_STL_FILES
        "${CMAKE_CURRENT_BINARY_DIR}/models/inspectorbot/meshes/base_link.stl"
        "${CMAKE_CURRENT_BINARY_DIR}/models/inspectorbot/meshes/propeller.stl"
    )

    # Add InspectorBot mesh generation target
    add_custom_target(inspectorbot_meshes ALL DEPENDS ${INSPECTORBOT_STL_FILES})

    # Install InspectorBot meshes
    install(
        FILES ${INSPECTORBOT_STL_FILES}
        DESTINATION share/${PROJECT_NAME}/models/inspectorbot/meshes
    )
else()
    message(WARNING "OpenSCAD not found. Mesh generation will be skipped.")
endif()

# Install directories
install(
    DIRECTORY 
        launch
        models
        config
        worlds
    DESTINATION
        share/${PROJECT_NAME}
)

# Potential Future Enhancement - custom dynamics, commanding/control, or sensor model implementations
# add_library(bimbot_dyn src/Dynamics.cpp)
# add_executable(bimbot_node src/BimbotNode.cpp)
# ament_target_dependencies(bimbot_node rclcpp std_msgs)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package
ament_package()
