# create an executable
SET (DEMO1_SOURCE ${PROJECT_SOURCE_DIR}/demo1/main.cpp)
ADD_EXECUTABLE (demo1 ${DEMO1_SOURCE})
TARGET_LINK_LIBRARIES (demo1 ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY 
	world1.urdf
	v\ groove\ butt\ welding\ piece.mtl
	v\ groove\ butt\ welding\ piece.obj
	DESTINATION ${RESOURCE_DIR}
)
