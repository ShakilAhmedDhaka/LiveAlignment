IF(WIN32)
    file (GLOB CAPTURE_HEADER "*.h")
    file(GLOB CAPTURE_SRC "*.cpp" "*.c")
ENDIF()
#message( ${CAPTURE_HEADER} )
#message( ${CMAKE_CURRENT_SOURCE_DIR} )
message( "${CMAKE_SOURCE_DIR}/lib/AzureKinect/sdk/include" )
message("${HELPERS_LIBS}")

message( ${CAPTURE_SRC} )
add_library(
    capture_lib 
    ${CAPTURE_SRC} 
    ${CAPTURE_HEADER} 
)


# adding kinect azure libraries and headers
include_directories("${CMAKE_SOURCE_DIR}/lib/AzureKinect/sdk/include")
link_directories("${CMAKE_SOURCE_DIR}/lib/AzureKinect/sdk/windows-desktop/amd64/release/lib")


IF(WIN32 AND MSVC)
    target_link_libraries (
    capture_lib
    "k4a.lib"
    "k4arecord.lib"
    ${HELPER_LIBS}
)
ENDIF()


SET_TARGET_PROPERTIES(capture_lib PROPERTIES LINKER_LANGUAGE CXX)