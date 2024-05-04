cmake_minimum_required(VERSION 3.10)
project(legged_sirius_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        legged_hw
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        "lcm-types/cpp"
        "/usr/local/include/lcm/"  # lcm includes
        "/usr/local/include/eigen3"  # lcm includes
)

set(EXTRA_LIBS lcm)

## Declare cpp executables
add_executable(${PROJECT_NAME}
        src/legged_sirius_hw.cpp
        src/SiriusHW.cpp
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
        )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(PROGRAMS scripts/make_types.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
