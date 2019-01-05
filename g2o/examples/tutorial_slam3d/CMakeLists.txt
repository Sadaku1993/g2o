include_directories(${CSPARSE_INCLUDE_DIR})

add_library(tutorial_slam3d_library ${G2O_LIB_TYPE}
    vertex_se3.h vertex_se3.cpp
    vertex_pointxyz.h vertex_pointxyz.cpp
    edge_se3.h edge_se3.cpp
    edge_se3_pointxyz.h edge_se3_pointxyz.cpp
    parameter_se3_offset.h parameter_se3_offset.cpp
    g2o_types_slam3d_api.h
)

set_target_properties(tutorial_slam3d_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}tutorial_slam3d)

target_link_libraries(tutorial_slam3d_library core solver_csparse)

add_executable(tutorial_slam3d
    tutorial_slam3d.cpp
)

set_target_properties(tutorial_slam3d PROPERTIES OUTPUT_NAME tutorial_slam3d${EXE_POSTFIX})

target_link_libraries(tutorial_slam3d tutorial_slam3d_library)
