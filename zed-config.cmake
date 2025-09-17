# ===================================================================================
#  The ZED CMake configuration file
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(zed REQUIRED)
#    INCLUDE_DIRECTORIES(${ZED_INCLUDE_DIRS})
#    link_directories(${ZED_LIBRARY_DIR})
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${ZED_LIBRARIES})
#
#    This file will define the following variables:
#      - ZED_LIBRARIES          : The list of libraries to links against.
#      - ZED_LIBRARY_DIR        : The directory where lib files are.
#      - ZED_INCLUDE_DIRS       : The ZED include directories.
#      - ZED_CUDA_VERSION       : The CUDA version required by the ZED SDK.
#      - CUDA_DEP_LIBRARIES_ZED : The CUDA dependencies libraries used by the ZED SDK.
# ===================================================================================
 
SET(ZED_FOUND FALSE CACHE BOOL "" FORCE)

IF(CMAKE_CL_64)
    IF("${ZED_DIR}" STREQUAL "")
        SET(ZED_DIR $ENV{ZED_SDK_ROOT_DIR})
    ENDIF()

    SET(ZED_INCLUDE_DIRS ${ZED_DIR}/include)
    SET(ZED_LIBRARY_DIR ${ZED_DIR}/lib)

    IF(ZED_INCLUDE_DIRS)
  
		SET(ZED_LIBRARIES_DEBUG ${ZED_LIBRARY_DIR}/sl_zed64d.lib)
		IF(EXISTS ${ZED_LIBRARIES_DEBUG})
		    add_definitions(-DALLOW_BUILD_DEBUG)
		ELSE()
		    SET(ZED_LIBRARIES_DEBUG ${ZED_LIBRARY_DIR}/sl_zed64.lib)
		ENDIF()
	 
        SET(ZED_LIBRARIES_RELEASE ${ZED_LIBRARY_DIR}/sl_zed64.lib) 
		SET(ZED_LIBRARIES debug ${ZED_LIBRARIES_DEBUG} optimized ${ZED_LIBRARIES_RELEASE})
		
        SET(ZED_FOUND TRUE CACHE BOOL "" FORCE)

        SET(ZED_CUDA_VERSION 11)
        
        SET(GLUT_INCLUDE_DIR ${ZED_DIR}/dependencies/freeglut_2.8/include CACHE BOOL "" FORCE)
        SET(GLEW_INCLUDE_DIR ${ZED_DIR}/dependencies/glew-1.12.0/include CACHE BOOL "" FORCE)
        SET(GLUT_LIBRARY_DIRS ${ZED_DIR}/dependencies/freeglut_2.8/x64 CACHE BOOL "" FORCE)
        SET(GLEW_LIBRARY_DIRS ${ZED_DIR}/dependencies/glew-1.12.0/x64 CACHE BOOL "" FORCE)
        SET(GLUT_LIBRARY "${GLUT_LIBRARY_DIRS}/freeglut.lib" CACHE BOOL "" FORCE)
        SET(GLEW_LIBRARIES "${GLEW_LIBRARY_DIRS}/glew32.lib" CACHE BOOL "" FORCE)
        SET(OpenCV_DIR ${ZED_DIR}/dependencies/opencv_3.1.0 CACHE BOOL "" FORCE)
    ENDIF()
ELSE()
    message(FATAL_ERROR "You've selected the 32bit version of ${CMAKE_GENERATOR}. \n Please delete the cache (file->Delete Cache) and use the 64bit version. (${CMAKE_GENERATOR} Win64)")
ENDIF()
