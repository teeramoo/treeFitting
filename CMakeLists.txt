cmake_minimum_required(VERSION 2.8)

project(treeFit)
#Set Cmake standard to c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#Set PCL package
find_package(PCL 1.8 REQUIRED)
IF(PCL_FOUND)
	message(STATUS "found PCL at ${PCL_LIBRARY_DIRS}")
ENDIF(PCL_FOUND)

find_package(OpenCV 3.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
		)

add_library(${PROJECT_NAME} SHARED
		src/PlaneProcessor.cpp
        src/Viewer.cpp
        src/NormalEstimator.cpp
		src/CylinderProcessor.cpp
	#	src/HeightEstimator.cpp

		include/PlaneProcessor.h
        include/Viewer.h
        include/NormalEstimator.h
		include/CylinderProcessor.h
	#	include/HeightEstimator.h
		)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/)

#add_executable (treeFit treeFit.cpp src/planeSegmentation.cpp src/planeSegmentation.h)
add_executable (treeFitting treeFit_cylindersearch.cpp)

target_link_libraries(treeFitting ${PROJECT_NAME} ${PCL_LIBRARIES} ${OCTAVE_LIBRARIES} ${OpenCV_LIBS})





### Legacy code
#set(MKOCTFILE "mkoctfile")
#set(OCTARG "--link-stand-alone")
#add_custom_target("treeFit")
#add_custom_command(TARGET treeFit
#   PRE_BUILD
#   COMMAND ${MKOCTFILE} ${OCTARG}
#   "${CMAKE_CURRENT_SOURCE_DIR}/treeFit.cpp" -o treeFit
#	)
