set(LIBRARY_NAME path_planning)

add_library(${LIBRARY_NAME}
  SHARED
  path_planning.cpp
  )

target_include_directories(${LIBRARY_NAME}
  PUBLIC ${PROJECT_SOURCE_DIR}/src/Matrix
  PUBLIC ${PROJECT_SOURCE_DIR}/src/path_planning
  )

target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} Matrix)
