include(${CMAKE_CURRENT_SOURCE_DIR}/../../max-sdk-base/script/max-pretarget.cmake)

#############################################################
# MAX EXTERNAL
#############################################################

include_directories( 
	"${MAX_SDK_INCLUDES}"
	"${MAX_SDK_MSP_INCLUDES}"
	"${MAX_SDK_JIT_INCLUDES}"
)

file(GLOB PROJECT_SRC
     "*.h"
	 "*.c"
     "*.cpp"
)
add_library( 
	${PROJECT_NAME} 
	MODULE
	${PROJECT_SRC}
)
# sample rate changes .c and .h files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/SRC)

#########################
# ADDING LSL DEPENDENCIES
#########################

find_library(
	LSL
	NAMES lslver liblsl liblsl.2 liblsl.1.16.0
	HINTS "${CMAKE_SOURCE_DIR}/include/lib"
	NO_DEFAULT_PATH
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/include)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/lib)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/bin)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/lib)
# target_link_libraries(mydsp_tilde "liblsl")

# target_link_libraries(mydsp_tilde ${LSL})

# the following lsl.lib linking ideas were taken from here:
# https://github.com/sccn/labstreaminglayer/issues/29

# set(LSL_DIR ${CMAKE_SOURCE_DIR}/include/lib)


# If you use find_package(OpenCV), 
# then set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/third_party/opencv) before it. 
# Alternatively you can use direct paths: include_directories(${CMAKE_SOURCE_DIR}/third_party/opencv/include)
#  and corresponded link_directories(${CMAKE_SOURCE_DIR}/third_party/opencv/lib) call. 

# set(LSL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
# find_package(LSL) #this finds the package in my global installs for some reason

# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/cmake/LSL)

# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/lib/cmake")
# find_package(LSL REQUIRED
#      HINTS ${LSL_INSTALL_ROOT}
#       "${CMAKE_CURRENT_LIST_DIR}/../../LSL/liblsl/build/"
#       "${CMAKE_CURRENT_LIST_DIR}/../../LSL/liblsl/build/install"
#       PATH_SUFFIXES share/LSL
# )

# set_target_properties(LSL::lsl PROPERTIES LINK_SEARCH_START_STATIC 1)
# set_target_properties(LSL::lsl PROPERTIES LINK_SEARCH_END_STATIC 1)
# # target_link_libraries(LSL::lsl ${CMAKE_CURRENT_SOURCE_DIR}/include/lib/liblsl.1.16.0.dylib)

# target_link_libraries(${PROJECT_NAME} PRIVATE LSL::lsl)


include(${CMAKE_CURRENT_SOURCE_DIR}/../../max-sdk-base/script/max-posttarget.cmake)
