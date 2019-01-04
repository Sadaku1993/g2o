INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
ADD_EXECUTABLE(tutorial_slam3d
  tutorial_slam3d.cpp
)

SET_TARGET_PROPERTIES(tutorial_slam3d PROPERTIES OUTPUT_NAME tutorial_slam3d${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(tutorial_slam3d tutorial_slam3d_library)
