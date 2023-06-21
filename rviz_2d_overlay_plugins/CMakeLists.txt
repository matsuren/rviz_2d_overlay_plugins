cmake_minimum_required(VERSION 3.8)
project(rviz_2d_overlay_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(std_msgs REQUIRED)
find_package(dump_messages REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


add_library(
        ${PROJECT_NAME} SHARED
        src/overlay_text_display.cpp
        src/overlay_utils.cpp
        src/pie_chart_display.cpp
        src/performance.cpp
        src/teleopwidget.cpp
        src/teleopwidget.ui
        src/touchwidget.cpp
)

target_include_directories(
        ${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
        ${Qt5Widgets_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME} PUBLIC
        rviz_ogre_vendor::OgreMain
        rviz_ogre_vendor::OgreOverlay
        Qt5::Widgets
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "${PROJECT_NAME}_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(
        ${PROJECT_NAME}
        PUBLIC
        rviz_common
        rviz_rendering
        rviz_2d_overlay_msgs
        std_msgs
        dump_messages
)

#ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rviz_common
        rviz_ogre_vendor
        dump_messages
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

#install(
#        DIRECTORY include/
#        DESTINATION include
#
install(DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)

ament_package()