INCLUDE(${QT_USE_FILE})

# ${QT_INCLUDE_DIR} does not appear to work; therefore have to
# manually rig it; also have to add QtCore directly because qglviewer
# doesn't seem to handle the Qt directories properly

QT4_WRAP_CPP(UI_SOURCES glo_selection_box.h)

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR}/.. ${QGLVIEWER_INCLUDE_DIR})

ADD_LIBRARY(alu_glviewer_objects
  glo_selection_box.h glo_selection_box.cpp
  glo_primitives.h glo_primitives.cpp
  glo_empty_box.h glo_empty_box.cpp
  glo_sparse_pointcloud.h glo_sparse_pointcloud.cpp
  glo_texturedPlane.h glo_texturedPlane.cpp
)

  IF(${PROJECT_NAME}_BUILD_SURFACEPRIMITIVES)
  ADD_LIBRARY(alu_glviewer_objects_scm
    glo_scm.h glo_scm.cpp
    glo_scfm.h glo_scfm.cpp
  )
  ENDIF(${PROJECT_NAME}_BUILD_SURFACEPRIMITIVES)



SET_TARGET_PROPERTIES(alu_glviewer_objects PROPERTIES OUTPUT_NAME ${LIB_PREFIX}alu_glviewer_objects)

TARGET_LINK_LIBRARIES(alu_glviewer_objects)

INSTALL(TARGETS alu_glviewer_objects
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ssa/alu_glviewer_objects)
