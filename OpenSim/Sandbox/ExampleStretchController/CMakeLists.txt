add_executable(modelBasedController
    modelbasedStretchController.cpp
    HelperMethods.h
    ModelBasedController.h)

target_link_libraries(modelBasedController osimTools)

set_target_properties(modelBasedController PROPERTIES
    PROJECT_LABEL "ModelBased Controller"
    FOLDER "Examples/ExampleStretchController"
    EXCLUDE_FROM_ALL TRUE
    )

add_executable(testbedStretchController
    testbedStretchController.cpp
    HelperMethods.h
    StretchController.h)

target_link_libraries(testbedStretchController osimTools)

set_target_properties(testbedStretchController PROPERTIES
    PROJECT_LABEL "Testbed Stretch Controller"
    FOLDER "Examples/ExampleStretchController"
    EXCLUDE_FROM_ALL TRUE
    )

add_executable(tugofwarStretchController
    tugofwarStretchController.cpp
    HelperMethods.h
    StretchController.h)

target_link_libraries(tugofwarStretchController osimTools)

set_target_properties(tugofwarStretchController PROPERTIES
    PROJECT_LABEL "TugofWar Stretch Controller"
    FOLDER "Examples/ExampleStretchController"
    EXCLUDE_FROM_ALL TRUE
    )
	
add_executable(armStretchController
    armStretchController.cpp
    HelperMethods.h
    StretchController.h)

target_link_libraries(armStretchController osimTools)

set_target_properties(armStretchController PROPERTIES
    PROJECT_LABEL "Arm Stretch Controller"
    FOLDER "Examples/ExampleStretchController"
    EXCLUDE_FROM_ALL TRUE
    )

set(MODEL_FILES "arm26.osim")

configure_file(${MODEL_FILES} ${CMAKE_CURRENT_BINARY_DIR}/${MODEL_FILES} COPYONLY)

	
	