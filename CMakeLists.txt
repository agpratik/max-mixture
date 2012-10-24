CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(g2o_max_mixture)

SET(CMAKE_DEBUG true)
SET(MIXG2O_LIB_PREFIX g2o_max_mix_)

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

#Set G2O_ROOT env variable to help find g2o package. Eigen3 find package uses g2o cmake module
set(CMAKE_MODULE_PATH  ${CMAKE_MODULE_PATH} $ENV{G2O_ROOT}/cmake_modules) 
message(STATUS ${CMAKE_MODULE_PATH})
FIND_PACKAGE(G2O REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -mfpmath=sse -msse3 -fPIC -O3 -DNDEBUG")
set(CMAKE_LD_FLAGS_RELEASE "${CMAKE_LD_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -ggdb")


if(G2O_FOUND)
    if(G2O_INCLUDE_DIR)
        message(STATUS "G2O include dir is ${G2O_INCLUDE_DIR}")
    else(G2O_INCLUDE_DIR)
        message(STATUS "G2O include dir not found")
    endif()
    
    if(G2O_TYPES_SLAM2D)
        message(STATUS "G2O_TYPES_SLAM2D lib is ${G2O_TYPES_SLAM2D}")
    else(G2O_TYPES_SLAM2D)
        message(STAUS "G2O_TYPES_SLAM2D Not found")
    endif()
    
    if(G2O_TYPES_SLAM3D)
        message(STATUS "G2O_TYPES_SLAM3D lib is ${G2O_TYPES_SLAM3D}")
    else(G2O_TYPES_SLAM3D)
        message(STAUS "G2O_TYPES_SLAM3D not found")
    endif()
   
    if(G2O_STUFF_LIBRARY)
        message(STATUS " G2O_STUFF_LIBS lib is ${G2O_STUFF_LIBRARY}")
    else(G2O_STUFF_LIBRARY)
        message(STAUS "G2O_STUFF_LIBS not found")
    endif()

    if(G2O_CORE_LIBRARY)
        message(STATUS "G2O_CORE_LIBS lib is ${G2O_CORE_LIBRARY}")
    else(G2O_CORE_LIBS)
        message(STAUS "G2O_CORE_LIBS not found")
    endif()


    #required for g2o/config.h 
    #It assumes the outof core built was used. 
    set(G2O_BUILD_DIR ${G2O_INCLUDE_DIR}/build)

    set(G2O_ALL_LIBS ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
    include_directories(${G2O_INCLUDE_DIR} ${G2O_BUILD_DIR} ${EIGEN3_INCLUDE_DIR})
    MESSAGE(STATUS "Found g2o")
    add_subdirectory(src)
else()
    MESSAGE(STATUS "Didnot find g2o") 
endif()



