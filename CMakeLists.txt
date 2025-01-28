cmake_minimum_required(VERSION 3.5.0)
project(aligator_mpc VERSION 0.1.0 LANGUAGES C CXX)

find_package(pinocchio REQUIRED)
find_package(aligator REQUIRED)

include_directories(include)

file(GLOB SOURCE_FILES 
    "src/*.cpp"
)

add_executable(aligator_mpc ${SOURCE_FILES})
target_link_libraries(aligator_mpc pinocchio::pinocchio)
target_link_libraries(aligator_mpc aligator::aligator)
