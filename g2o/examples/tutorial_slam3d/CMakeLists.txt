include_directories(${CSPARSE_INCLUDE_DIR})

add_executable(tutorial_slam3d
    tutorial_slam3d.cpp
)

set_target_properties(tutorial_slam3d PROPERTIES OUTPUT_NAME tutorial_slam3d${EXE_POSTFIX})

target_link_libraries(tutorial_slam3d 
  core
  solver_csparse
  types_slam3d
  ${OPENGL_LIBRARIES}
)
