# This is the cmakelists.txt if you want to compile the C++ project locally, without building the bindings or using sckikit-build

# run cmake -S . -B build
# run cmake --build build
# from the src/ directory. An executable called 'main' will now be available in the build/ directory

cmake_minimum_required(VERSION 3.16)
project(main)
message("Development mode is on")

file(GLOB_RECURSE SRC_FILES main.cpp ur_analytic_ik/*.cpp ur_analytic_ik/*.hh)

find_package(Eigen3 REQUIRED)

link_directories(${Eigen_INCLUDE_DIRS})

#add_executable(${PROJECT_NAME} ${SRC_FILES}    ur_analytic_ik/utils.h) #The name of the cpp file and its path can vary
add_executable(${PROJECT_NAME} "main.cpp"
        ur_analytic_ik/utils.h) #The name of the cpp file and its path can vary

target_link_libraries(${PROJECT_NAME}
        ${Eigen_LIBRARIES}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
