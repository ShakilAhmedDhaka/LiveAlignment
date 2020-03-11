IF(WIN32)
    file (GLOB HELPER_HEADER "*.h")
    file(GLOB HELPER_SRC "*.cpp" "*.c")
ENDIF()
#message( ${CAPTURE_HEADER} )
#message( ${CMAKE_CURRENT_SOURCE_DIR} )


add_library(
    helper_lib 
    ${HELPER_SRC} 
    ${HELPER_HEADER} 
)


# adding kinect azure libraries and headers
include_directories("${CMAKE_SOURCE_DIR}/lib/AzureKinect/sdk/include")
link_directories("${CMAKE_SOURCE_DIR}/lib/AzureKinect/sdk/windows-desktop/amd64/release/lib")

message("GLOBAL HELPER MUST NOT DEPEND ON ANY OTHER LOCAL LIBRARIES")
IF(WIN32 AND MSVC)
    target_link_libraries (
    helper_lib
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
ENDIF()


SET_TARGET_PROPERTIES(helper_lib PROPERTIES LINKER_LANGUAGE CXX)