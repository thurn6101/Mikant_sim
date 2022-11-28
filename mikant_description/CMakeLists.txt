cmake_minimum_required(VERSION 3.5)

project(mikant_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files("urdf/Catamaran_Sailboat.xacro" INSTALL DESTINATION urdf/)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME})

install(FILES model.config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
