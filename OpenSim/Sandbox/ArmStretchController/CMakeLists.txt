add_executable(armStretchController
    exampleStretchController.cpp
    helperMethods.h
    StretchController.h)

target_link_libraries(armStretchController osimTools)

set_target_properties(armStretchController PROPERTIES
    PROJECT_LABEL "Arm Stretch Controller"
    FOLDER "Examples"
    EXCLUDE_FROM_ALL TRUE
    )

add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/arm26.osim"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/arm26.osim"
    COMMAND ${CMAKE_COMMAND} -E copy
    "${CMAKE_CURRENT_SOURCE_DIR}/arm26.osim"
    "${CMAKE_CURRENT_BINARY_DIR}")