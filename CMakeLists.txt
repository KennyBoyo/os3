project(os3)

cmake_minimum_required(VERSION 2.8.6)

# Settings.
# ---------
set(TARGET testOS3 CACHE TYPE STRING)
set(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
        CACHE PATH "Top-level directory of OpenSim install")

# C++ Compiler Version
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Link Opensim
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")
if(${CMAKE_CURRENT_SOURCE_DIR})
    set(ROSBRIDGECPP_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rosbridgecpp")
else()
    set(ROSBRIDGECPP_SRC_DIR "ROSBRIDGECPP_NOT_FOUND")
endif()
include_directories(${OpenSim_INCLUDE_DIRS})
link_directories(${OpenSim_LIB_DIR})

# Link Rosbridge
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rosbridgecpp)
    set(ROSBRIDGECPP_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rosbridgecpp")
else()
    set(ROSBRIDGECPP_SRC_DIR "ROSBRIDGECPP_NOT_FOUND")
endif()


message(STATUS ${ROSBRIDGECPP_SRC_DIR})
include_directories(${ROSBRIDGECPP_SRC_DIR})
add_subdirectory(${ROSBRIDGECPP_SRC_DIR} rosbridgecpp)

########################
# include os3 includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES src/*.h src/*.cpp )

add_executable(${TARGET} ${SOURCE_FILES})

target_link_libraries(${TARGET} ${OpenSim_LIBRARIES})
target_link_libraries(${TARGET} rosbridgecpp)


#copy osim models to Models folder in running directory
file(GLOB OSIM_MODELS src/Models/*.osim)

file(MAKE_DIRECTORY build/Models)
#add_custom_command(
#    TARGET ${TARGET}
#    COMMAND ${CMAKE_COMMAND}
#    ARGS -E make_directory
#    ${CMAKE_BINARY_DIR}/Models
#    )   

set(osimModelName none CACHE TYPE STRING)
foreach(osimModel ${OSIM_MODELS})
    
    get_filename_component(osimModelName ${osimModel} NAME)
    #cmake_path(GET osimModelName, ${osimModel}, FILENAME)
    message(STATUS name: ${osimModelName})
    add_custom_command(
        TARGET ${TARGET}
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${osimModel}
        ${CMAKE_BINARY_DIR}/Models/${osimModelName})
    message(STATUS hi: ${CMAKE_BINARY_DIR}/Models/${osimModelName})
endforeach(osimModel)


# This block copies the additional files into the running directory
# For example vtp, obj files. Add to the end for more extentions
file(MAKE_DIRECTORY build/Models/Geometry)
file(GLOB DATA_FILES src/Models/Geometry/*.vtp src/Models/Geometry/*.obj)
foreach(dataFile ${DATA_FILES})
    add_custom_command(
        TARGET ${TARGET}
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${dataFile}
        ${CMAKE_BINARY_DIR}/Models/Geometry)
endforeach(dataFile)
