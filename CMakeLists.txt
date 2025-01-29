cmake_minimum_required(VERSION 3.5.0)
project(aligator_mpc VERSION 0.1.0 LANGUAGES C CXX)

find_package(pinocchio REQUIRED)
find_package(aligator REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

file(GLOB SOURCE_FILES 
    "src/*.cpp"
)

add_executable(aligator_mpc ${SOURCE_FILES})
target_link_libraries(aligator_mpc pinocchio::pinocchio)
target_link_libraries(aligator_mpc aligator::aligator)
target_link_libraries(aligator_mpc yaml-cpp::yaml-cpp)

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/utils/project_path.h.in"
  "${PROJECT_SOURCE_DIR}/include/utils/project_path.h" @ONLY
)