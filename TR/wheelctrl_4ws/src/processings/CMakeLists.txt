set(LIBRARY_NAME processings)

add_library(${LIBRARY_NAME}
  SHARED
  processings.cpp
  )

target_include_directories(${LIBRARY_NAME}
  PUBLIC ${PROJECT_SOURCE_DIR}/${LIBRARY_NAME}
  )

target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})
