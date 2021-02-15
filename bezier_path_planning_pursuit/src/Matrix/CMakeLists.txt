set(LIBRARY_NAME Matrix)

add_library(${LIBRARY_NAME}
  SHARED
  matrix.cpp
  )

target_include_directories(${LIBRARY_NAME}
  PUBLIC ${PROJECT_SOURCE_DIR}/src/Matrix
  )

target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})
