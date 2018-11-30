CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(FFLD)

# The build options
OPTION(FFLD_HOGPYRAMID_DOUBLE "Use doubles instead of floats (slower, uses twice more memory, and the increase in precision is not necessarily useful)." OFF)
OPTION(FFLD_HOGPYRAMID_EXTRA_FEATURES "Use extra features (LBP + color) in addition to HOG." OFF)
OPTION(FFLD_MODEL_3D "Allow parts to also deform across scales." OFF)
OPTION(FFLD_MIXTURE_STANDARD_CONVOLUTION "Use standard convolutions instead of the optimized Fourier ones." OFF)

# Select a default build configuration if none was chosen
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()


option (TESTS "Compile tests" ON) 
include_directories(./include)

IF(TESTS)
	SET(src_pyramid src/test_pyramid.cpp )#GSHOTPyramid.h GSHOTPyramid.cpp)
	SET(src_model src/test_model.cpp )#Model.h Model.cpp GSHOTPyramid.h GSHOTPyramid.cpp typedefs.h)
	SET(src_mixture src/test_mixture.cpp )#Mixture.h Mixture.cpp Model.h Model.cpp LBFGS.h LBFGS.cpp GSHOTPyramid.h GSHOTPyramid.cpp Object.h Object.cpp Scene.h Scene.cpp Rectangle.h Rectangle.cpp)
	SET(create_scene src/createscene.cpp )#Mixture.h Mixture.cpp Model.h Model.cpp LBFGS.h LBFGS.cpp GSHOTPyramid.h GSHOTPyramid.cpp Object.h Object.cpp Scene.h Scene.cpp Rectangle.h Rectangle.cpp)
	SET(main src/main.cpp )#Mixture.h Mixture.cpp Model.h Model.cpp LBFGS.h LBFGS.cpp GSHOTPyramid.h GSHOTPyramid.cpp Object.h Object.cpp Scene.h Scene.cpp Rectangle.h Rectangle.cpp)
	SET(test_training src/test_training.cpp )#Mixture.h Mixture.cpp Model.h Model.cpp LBFGS.h LBFGS.cpp GSHOTPyramid.h GSHOTPyramid.cpp Object.h Object.cpp Scene.h Scene.cpp Rectangle.h Rectangle.cpp)
	ADD_EXECUTABLE(tests ${main} include/tensor3d.h include/EMD_DEFS.hpp include/emd_hat.hpp include/emd_hat_impl.hpp 
													include/min_cost_flow.hpp include/flow_utils.hpp include/viewer.h 
													include/Mixture.h src/Mixture.cpp include/Model.h src/Model.cpp 
													include/LBFGS.h src/LBFGS.cpp include/GSHOTPyramid.h src/GSHOTPyramid.cpp 
													include/Object.h src/Object.cpp include/Scene.h src/Scene.cpp 
													include/Rectangle.h src/Rectangle.cpp)
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
	#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
	
ELSE()
	# Also list the headers so that they are displayed along the .cpp files in the IDE
	SET(HEADERS HOGPyramid.h Intersector.h JPEGImage.h LBFGS.h Mixture.h Model.h Object.h Patchwork.h Rectangle.h Scene.h SimpleOpt.h)
	SET(SOURCES HOGPyramid.cpp JPEGImage.cpp LBFGS.cpp Mixture.cpp Model.cpp Object.cpp Patchwork.cpp Rectangle.cpp Scene.cpp)
	ADD_EXECUTABLE(train ${HEADERS} ${SOURCES} train.cpp)
	ADD_EXECUTABLE(test ${HEADERS} ${SOURCES} test.cpp)
ENDIF()


# Define the options
IF(FFLD_HOGPYRAMID_DOUBLE)
  MESSAGE("Use doubles instead of floats.")
  ADD_DEFINITIONS(-DFFLD_HOGPYRAMID_DOUBLE)
ENDIF()

IF(FFLD_HOGPYRAMID_EXTRA_FEATURES)
  MESSAGE("Use extra features (LBP + color) in addition to HOG.")
  ADD_DEFINITIONS(-DFFLD_HOGPYRAMID_EXTRA_FEATURES)
ENDIF()

IF(FFLD_MODEL_3D)
  MESSAGE("Allow parts to also deform across scales.")
  ADD_DEFINITIONS(-DFFLD_MODEL_3D)
ENDIF()

IF(FFLD_MIXTURE_STANDARD_CONVOLUTION)
  MESSAGE("Use standard convolutions instead of the optimized Fourier ones.")
  ADD_DEFINITIONS(-DFFLD_MIXTURE_STANDARD_CONVOLUTION)
ENDIF()

FIND_PACKAGE(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
IF(NOT TESTS)
	target_link_libraries(train ${PCL_LIBRARIES})
	target_link_libraries(test ${PCL_LIBRARIES})
ENDIF()

# There are no CMake Eigen package, so find it ourselves
FILE(GLOB EIGEN_ARCHIVE "/usr/include/eigen*")
FIND_PATH(EIGEN_INCLUDE_DIR Eigen ${EIGEN_ARCHIVE} .)
IF(NOT EIGEN_INCLUDE_DIR)
  MESSAGE(FATAL_ERROR "Could not find Eigen.")
ENDIF()
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

# There are no CMake FFTW package, so find it ourselves
FIND_PATH(FFTW3_INCLUDE_DIR fftw3.h)
IF(FFLD_HOGPYRAMID_DOUBLE)
  FIND_LIBRARY(FFTW3_LIBRARIES fftw3)
ELSE()
  FIND_LIBRARY(FFTW3_LIBRARIES fftw3f)
ENDIF()
IF(NOT FFTW3_INCLUDE_DIR OR NOT FFTW3_LIBRARIES)
  MESSAGE(FATAL_ERROR "Could not find fftw3.")
ENDIF()
INCLUDE_DIRECTORIES(${FFTW3_INCLUDE_DIR})
IF(NOT TESTS)
	TARGET_LINK_LIBRARIES(train ${FFTW3_LIBRARIES})
	TARGET_LINK_LIBRARIES(test ${FFTW3_LIBRARIES})
ENDIF()

FIND_PACKAGE(JPEG REQUIRED)
IF(JPEG_FOUND)
  INCLUDE_DIRECTORIES(${JPEG_INCLUDE_DIR})
  IF(NOT TESTS)
	  TARGET_LINK_LIBRARIES(train ${JPEG_LIBRARIES})
	  TARGET_LINK_LIBRARIES(test ${JPEG_LIBRARIES})
  ENDIF()
ENDIF()

FIND_PACKAGE(LibXml2 REQUIRED)
IF(LIBXML2_FOUND)
  INCLUDE_DIRECTORIES(${LIBXML2_INCLUDE_DIR})
  IF(NOT TESTS)
	  TARGET_LINK_LIBRARIES(train ${LIBXML2_LIBRARIES})
	  TARGET_LINK_LIBRARIES(test ${LIBXML2_LIBRARIES})
  ENDIF()
  ADD_DEFINITIONS(${LIBXML2_DEFINITIONS})
ENDIF()

#OPENMP
if(UNIX)
	#use openMP
	find_package( OpenMP REQUIRED)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(UNIX)

#OpenCV
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries (tests /usr/lib/x86_64-linux-gnu/libgomp.so.1)

add_definitions(-std=c++11 -fopenmp)



target_link_libraries(tests ${PCL_LIBRARIES} ${OpenCV_LIBS} ${FFTW3_LIBRARIES} ${LIBXML2_LIBRARIES})

set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "/usr/lib/x86_64-linux-gnu/")
