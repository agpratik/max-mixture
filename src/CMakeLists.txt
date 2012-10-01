CMAKE_MINIMUM_REQUIRED(VERSION 2.6)


set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#FIND_PACKAGE(Eigen3 REQUIRED)

#message(STATUS "g2o include directory is ${G2O_INCLUDE_DIR}")

ADD_LIBRARY(types_mixture SHARED
	    types_g2o_mixture.cpp
            edge_se2_mixture.h edge_se2_mixture.cpp
            edge_se2_pointxy_mixture.h edge_se2_pointxy_mixture.cpp
           edge_se3_mixture.h edge_se3_mixture.cpp) 

SET_TARGET_PROPERTIES(types_mixture PROPERTIES OUTPUT_NAME 
                      ${MIXG2O_LIB_PREFIX}core)
 
target_link_libraries(types_mixture ${G2O_ALL_LIBS} ${EIGEN3_LIBRARIES})
