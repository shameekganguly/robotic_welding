# create all executables
ADD_EXECUTABLE (demo5_controller demo5.cpp)
TARGET_LINK_LIBRARIES (demo5_controller ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY
	world5.urdf
	parts_curvilinear.mtl
	parts_curvilinear.obj
	parts_curvilinear_wall_only.mtl
	parts_curvilinear_wall_only.obj
	DESTINATION ${RESOURCE_DIR}
)
