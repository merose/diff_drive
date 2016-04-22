cmake_minimum_required(VERSION 2.8.3)
project(diff_drive)

find_package(catkin REQUIRED)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/your_first_rostest.test)
  add_rostest(tests/your_second_rostest.test)
endif()

catkin_package()

catkin_install_python(
    PROGRAMS
        nodes/diff_drive_odometry
        nodes/diff_drive_controller
        nodes/diff_drive_go_to_goal
	nodes/diff_drive_mock_robot
    DESTINATION ${CATKIN_PACKoAGE_BIN_DESTINATION})
