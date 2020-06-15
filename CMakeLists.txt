cmake_minimum_required(VERSION 3.5.1)
project(intelligent_process_behaviortree VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp_v3 REQUIRED)

add_library(${PROJECT_NAME}_nodes SHARED src/plan_freespace_nodes.cpp)
target_link_libraries(${PROJECT_NAME}_nodes PUBLIC ${BEHAVIOR_TREE_LIBRARY})
#target_link_libraries(${PROJECT_NAME}_nodes PRIVATE BT::behaviortree_cpp_v3)  // TODO: should work?
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_nodes PUBLIC ${behaviortree_cpp_v3_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME}_nodes PRIVATE  BT_PLUGIN_EXPORT )
set_target_properties(${PROJECT_NAME}_nodes PROPERTIES LIBRARY_OUTPUT_DIRECTORY bin )

add_executable(${PROJECT_NAME} src/plan_freespace.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} PUBLIC ${behaviortree_cpp_v3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${behaviortree_cpp_v3_LIBRARIES} ${PROJECT_NAME}_nodes)  # TODO: why doesn't ${BEHAVIOR_TREE_LIBRARY} work here?

INSTALL(TARGETS ${PROJECT_NAME}_nodes ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )

INSTALL( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
    DESTINATION include
    FILES_MATCHING PATTERN "*.h*")
