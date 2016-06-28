add_executable(tugofwarStretchController
    buildTugofwarModel.cpp
    exampleStretchController.cpp
    helperMethods.h
    StretchController.h)
target_link_libraries(tugofwarStretchController osimTools)
set_target_properties(tugofwarStretchController PROPERTIES
    PROJECT_LABEL "Tugofwar Stretch Controller"
    FOLDER "Examples"
    EXCLUDE_FROM_ALL TRUE
    )
