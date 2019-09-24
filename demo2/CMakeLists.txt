# create all executables
ADD_EXECUTABLE (demo2_controller demo2.cpp)
ADD_EXECUTABLE (visualizer visualizer.cpp)
ADD_EXECUTABLE (simulator simulator.cpp)
ADD_EXECUTABLE (haptic haptic_interface.cpp)
TARGET_LINK_LIBRARIES (demo2_controller ${COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (visualizer ${COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simulator ${COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (haptic ${COMMON_LIBRARIES})

ADD_EXECUTABLE (test_logger testLogger.cpp)
TARGET_LINK_LIBRARIES (test_logger ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY
	world2.urdf
	DESTINATION ${RESOURCE_DIR}
)
