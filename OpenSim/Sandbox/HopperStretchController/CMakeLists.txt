add_executable(exampleHopperStretchController
    exampleHopperDevice.cpp
    buildHopperModel.cpp
    buildDeviceModel.cpp
    defineDeviceAndController.h
    StretchController.h
    helperMethods.h)
target_link_libraries(exampleHopperStretchController osimTools)
set_target_properties(exampleHopperStretchController PROPERTIES
    PROJECT_LABEL "Hopper StretchController"
    FOLDER "Examples"
    EXCLUDE_FROM_ALL TRUE
    )
