cmake_minimum_required(VERSION 3.1.0)

project(Live_Alignment)

#set (CMAKE_PREFIX_PATH "C:/Qt5/5.12.2/msvc2017_64")

# will the cmake handle qt moc preprocessor automatically
set(CMAKE_AUTOMOC ON)

set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Widgets CORE REQUIRED)

find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})


find_package(PCL 1.10.0 REQUIRED)
# Additional Include Directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${FLANN_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
# Preprocessor Definitions
add_definitions(${PCL_DEFINITIONS})
# Additional Library Directories
link_directories(${PCL_LIBRARY_DIRS})

find_package(VTK REQUIRED)
# Load VTK Settings
include( ${VTK_USE_FILE} )


include_directories("${PROJECT_SOURCE_DIR}/lib/helpers")
add_subdirectory(lib/helpers)
set (HELPER_LIBS ${HELPER_LIBS} helper_lib)

include_directories("${PROJECT_SOURCE_DIR}/lib/visualization")
add_subdirectory(lib/visualization)
set (VISUALIZER_LIBS ${VISUALIZER_LIBS} visualizer_lib)

include_directories("${PROJECT_SOURCE_DIR}/lib/capture")
add_subdirectory(lib/capture)
set (CAPTURE_LIBS ${CAPTURE_LIBS} capture_lib)

include_directories("${PROJECT_SOURCE_DIR}/lib/apriltags")
add_subdirectory(lib/apriltags)
set(APRILTAGS_LIBS APRILTAG_LIBRARY)


include_directories("${PROJECT_SOURCE_DIR}/lib/alignment")
add_subdirectory(lib/alignment)
set (ALIGNMENT_LIBS ${ALIGNMENT_LIBS} alignment_lib)


include_directories("${PROJECT_SOURCE_DIR}/lib/pthreads-win32")
add_subdirectory(lib/pthreads-win32)



# adding kinect azure libraries and headers
include_directories("${CMAKE_SOURCE_DIR}/lib/AzureKinect/sdk/include")
link_directories("${CMAKE_SOURCE_DIR}/lib/AzureKinect/sdk/windows-desktop/amd64/release/lib")





add_executable(
    capture_alignment
    main.cpp
    resources.qrc
    pclviewer.h
    pclviewer.cpp
    pclviewer.ui
)

target_link_libraries(
    capture_alignment
    Qt5::Widgets
	${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
	${CAPTURE_LIBS}
	${APRILTAGS_LIBS}
    ${ALIGNMENT_LIBS}
    ${VISUALIZER_LIBS}
)