if(DEFINED ENV{MVCAM_SDK_PATH})
    set(MVCAM_SDK_PATH $ENV{MVCAM_SDK_PATH} CACHE PATH "hikvision sdk path")
else()
    set(MVCAM_SDK_PATH /opt/MVS CACHE PATH "hikvision sdk path")
endif()

# find header
find_path(MVS_INCLUDE_DIR 
    NAMES MvCameraControl.h
    PATHS ${MVCAM_SDK_PATH}/include
)

# find libraries
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    find_library(MVS_LIBRARIES
        NAMES MvCameraControl
        PATHS ${MVCAM_SDK_PATH}/lib/64
    )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    find_library(MVS_LIBRARIES
        NAMES MvCameraControl
        PATHS ${MVCAM_SDK_PATH}/lib/aarch64
    )    
endif()


message(VERBOSE "MVCAM_SDK_PATH: ${MVCAM_SDK_PATH}")
message(VERBOSE "MVS_INCLUDE_DIR: ${MVS_INCLUDE_DIR}")
message(VERBOSE "MVS_LIBRARIES: ${MVS_LIBRARIES}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MVS
  FOUND_VAR MVS_FOUND
  REQUIRED_VARS
    MVS_INCLUDE_DIR
    MVS_LIBRARIES
)