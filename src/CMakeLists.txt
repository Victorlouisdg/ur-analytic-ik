# This is the cmakelists.txt when wanting to run cmake . locally without scikit build core.
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