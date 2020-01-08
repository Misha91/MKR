include_directories("${PROJECT_SOURCE_DIR}")

add_library(Gui gui.cpp)
target_link_libraries(Gui ${VTK_LIBRARIES})
