# In your 'CMakeLists.txt', assuming you've placed 'RustTarget.cmake' in a top-level 'cmake' subdir
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(RustTarget)
add_rust_target(rs41_test RELEASE)
