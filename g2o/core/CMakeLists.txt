ADD_LIBRARY(core
base_edge.h
base_binary_edge.h          hyper_graph_action.cpp
base_binary_edge.hpp        hyper_graph_action.h
base_multi_edge.h           hyper_graph.cpp
base_multi_edge.hpp         hyper_graph.h
base_unary_edge.h           linear_solver.h
base_unary_edge.hpp         marginal_covariance_cholesky.cpp
base_vertex.h               marginal_covariance_cholesky.h
base_vertex.hpp             matrix_structure.cpp
batch_stats.cpp             matrix_structure.h
batch_stats.h               openmp_mutex.h
block_solver.h              block_solver.hpp
parameter.cpp               parameter.h
cache.cpp                   cache.h
optimizable_graph.cpp       optimizable_graph.h
solver.cpp                  solver.h
creators.h                  optimization_algorithm_factory.cpp
estimate_propagator.cpp     optimization_algorithm_factory.h
estimate_propagator.h
factory.cpp                 optimization_algorithm_property.h
factory.h                   sparse_block_matrix.h
sparse_optimizer.cpp  sparse_block_matrix.hpp
sparse_optimizer.h
hyper_dijkstra.cpp hyper_dijkstra.h
parameter_container.cpp     parameter_container.h
optimization_algorithm.cpp optimization_algorithm.h
optimization_algorithm_with_hessian.cpp optimization_algorithm_with_hessian.h
optimization_algorithm_gauss_newton.cpp optimization_algorithm_gauss_newton.h
optimization_algorithm_levenberg.cpp optimization_algorithm_levenberg.h
optimization_algorithm_dogleg.cpp optimization_algorithm_dogleg.h
sparse_optimizer_terminate_action.cpp sparse_optimizer_terminate_action.h
jacobian_workspace.cpp jacobian_workspace.h
robust_kernel.cpp robust_kernel.h
robust_kernel_impl.cpp robust_kernel_impl.h
robust_kernel_factory.cpp robust_kernel_factory.h
)

generate_export_header (core PREFIX_NAME G2O_ EXPORT_FILE_NAME
  ${CMAKE_BINARY_DIR}/include/g2o/core/g2o_core_api.h
  EXPORT_MACRO_NAME CORE_API)

SET_TARGET_PROPERTIES(core PROPERTIES OUTPUT_NAME ${LIB_PREFIX}core)

TARGET_LINK_LIBRARIES(core stuff)

if (TARGET Eigen3::Eigen)
  target_link_libraries (core PUBLIC Eigen3::Eigen)
else (TARGET Eigen3::Eigen)
  target_include_directories (core SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
endif (TARGET Eigen3::Eigen)

target_include_directories (core PUBLIC
  $<INSTALL_INTERFACE:${_g2o_CMake_Base_INCLUDEDIR}>
  $<BUILD_INTERFACE:${g2o_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

target_compile_features (core
  PUBLIC cxx_rvalue_references cxx_constexpr cxx_nullptr
  PRIVATE cxx_auto_type
)

INSTALL(TARGETS core EXPORT g2o-targets
  RUNTIME DESTINATION ${_g2o_CMake_BINDIR}
  LIBRARY DESTINATION ${_g2o_CMake_LIBDIR}
  ARCHIVE DESTINATION ${_g2o_CMake_LIBDIR}
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} ${CMAKE_BINARY_DIR}/include/g2o/core/g2o_core_api.h
  DESTINATION ${_g2o_CMake_INCLUDEDIR}/core)
