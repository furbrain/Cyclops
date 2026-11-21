cmake_minimum_required(VERSION 3.8)
project(tactical_frog)

# The nodes are Python, but only an ament_cmake package can generate interfaces, so this builds both: the
# messages, services and actions in msg/, srv/ and action/, and the Python package in tactical_frog/
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Calibrate.action"
  "srv/SetTransform.srv"
  "srv/Beep.srv"
  "srv/BeepPreset.srv"
  "srv/SetMode.srv"
  "msg/Sync.msg"
  DEPENDENCIES geometry_msgs
)

# rosidl has already installed a Python package called tactical_frog (holding tactical_frog.msg, tactical_frog.srv
# and tactical_frog.action), and ament_python_install_package() refuses a second package of the same name, so the
# nodes' modules are copied into that same package instead
install(DIRECTORY tactical_frog/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# One executable per node, for "ros2 run tactical_frog <node>" and the launch files (what setup.py's
# console_scripts used to make). Each node's module needs a main().
set(NODES
  battery_status
  beeper
  button_monitor
  calibrate
  calibrate_cam_imu
  calibrate_dual
  led_indicator
  orb_make_settings
  orb_state_beeper
  process_monitor
  switcher
  tf2_dynamic
)
foreach(node ${NODES})
  file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/nodes/${node}" CONTENT
    "#!/usr/bin/python3\nimport sys\nfrom tactical_frog.${node} import main\nsys.exit(main())\n")
  install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/nodes/${node}" DESTINATION lib/${PROJECT_NAME})
endforeach()

# hidden files are skipped, as setup.py's glob did (config/ has held a stray named pipe, which hangs the install)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} PATTERN ".*" EXCLUDE)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
