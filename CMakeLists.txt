cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME MonoCalibration)
PROJECT(${PROJECTNAME})
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")

find_package(OpenCV)
find_package(Eigen3)

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
		    ${EIGEN3_INCLUDE_DIR})

# ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

# TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES})
#                                     ${YARP_LIBRARIES})


set(SHARED_DIR "${CMAKE_SOURCE_DIR}/../shared")

set(BOOST_LATEST_DIR            "${SHARED_DIR}/boost_1_65.1")
set(BOOST_LATEST_BIN_DIR        "${BOOST_LATEST_DIR}/stage/lib")
set(BOOST_LATEST_INCLUDE_DIR    "${BOOST_LATEST_DIR}/boost")
set(BOOST_SYSTEM                "${BOOST_LATEST_BIN_DIR}/libboost_system.so")
set(BOOST_FS                    "${BOOST_LATEST_BIN_DIR}/libboost_filesystem.so")
set(BOOST_THREAD                "${BOOST_LATEST_BIN_DIR}/libboost_thread.so")


include_directories(${CMAKE_SOURCE_DIR} ${BOOST_LATEST_INCLUDE_DIR} )

set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu/")
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

#add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

find_package(Boost 1.65.1 REQUIRED COMPONENTS system thread filesystem)

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})


if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(${PROJECTNAME} PUBLIC Boost::system Boost::filesystem Boost::thread ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})
# ${HYRISE_SQLPARSER})
endif()

























INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)





