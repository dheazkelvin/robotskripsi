cmake_minimum_required(VERSION 3.5)
project(my_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
# No need to find PyQt5, speech_recognition, pyaudio, numpy, scipy here

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/go_to_pose.py
  scripts/go_to_pose2.py
  scripts/go_to_pose3.py
  scripts/go_to_pose4.py
  scripts/go_to_pose5.py
  scripts/choose_goal.py
  scripts/qos_bridge.py
  scripts/voice_recognition.py
  scripts/sound_tester.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

