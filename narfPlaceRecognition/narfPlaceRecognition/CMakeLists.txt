#INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})


############
# GUI PART #
############
IF (BUILD_GUI_ELEMENTS)
    INCLUDE(${QT_USE_FILE})

    QT4_WRAP_UI(UI_HEADERS baseMainWidget.ui)
    QT4_WRAP_CPP(UI_SOURCES mainWidget.h)

    INCLUDE_DIRECTORIES(${QT_QTCORE_INCLUDE_DIR} ${QT_QTCORE_INCLUDE_DIR}/..)
    ADD_LIBRARY(narfPlaceRecognitionGui
        mainWidget.h mainWidget.cpp
        visMapCloud.h visMapCloud.cpp
        ${UI_HEADERS}
        ${UI_SOURCES}
        )
    TARGET_LINK_LIBRARIES(narfPlaceRecognitionGui
        alu_glviewer alu_glviewer_objects imageWidget  # Ais3dTools
        ${QT_LIBRARIES}
        ${QGLVIEWER_LIBRARY}
        ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
        )
    ADD_EXECUTABLE(narfPlaceRecognition narfPlaceRecognition.cpp)
    TARGET_LINK_LIBRARIES(narfPlaceRecognition
        ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}
        g2o_core g2o_types_slam3d g2o_solver_csparse #g2o_stuff g2o_types_icp g2o_core g2o_solver_cholmod
        basics
        spm_base spm_dictionary_selection kmeans++
        narfPlaceRecognitionLib
        narfPlaceRecognitionGui
        )
    ADD_DEPENDENCIES(narfPlaceRecognition g2o)

    ADD_EXECUTABLE(sebmain sebmain.cpp)
    TARGET_LINK_LIBRARIES(sebmain
        ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}
        g2o_core g2o_types_slam3d g2o_solver_csparse #g2o_stuff g2o_types_icp g2o_core g2o_solver_cholmod
        basics
        spm_base spm_dictionary_selection kmeans++
        narfPlaceRecognitionLib
        narfPlaceRecognitionGui
        )
    ADD_DEPENDENCIES(narfPlaceRecognition g2o)

    ##########
    # NO GUI #
    ##########
ELSE (BUILD_GUI_ELEMENTS)
    #NOTHING HERE YET
ENDIF (BUILD_GUI_ELEMENTS)
