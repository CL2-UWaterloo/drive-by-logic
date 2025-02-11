cmake_minimum_required(VERSION 3.5)
project(optimal_control)

find_package(ament_cmake REQUIRED)

# https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY scripts
    DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
    scripts/narrow_passage.py
    scripts/reach_avoid.py
    scripts/opensaysme.py
    scripts/needforspeed.py
    scripts/view_environments.py
    scripts/run_experiment.py
    scripts/initial_guess_gen.py
    scripts/reach_avoid_physical.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
ament_package()