INCLUDE(${QT_USE_FILE})

# ${QT_INCLUDE_DIR} does not appear to work; therefore have to
# manually rig it; also have to add QtCore directly because qglviewer
# doesn't seem to handle the Qt directories properly

QT4_WRAP_CPP(UI_SOURCES alu_glviewer.h alu_glviewer_object.h)

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR}/.. ${QGLVIEWER_INCLUDE_DIR})


ADD_LIBRARY(alu_glviewer 
  alu_glviewer.cpp
  alu_glviewer.h
  alu_glviewer_object.cpp
  alu_glviewer_object.h
  standard_camera.h
  ${UI_SOURCES}
)

SET_TARGET_PROPERTIES(alu_glviewer PROPERTIES OUTPUT_NAME ${LIB_PREFIX}alu_glviewer)

TARGET_LINK_LIBRARIES(alu_glviewer ${QGLVIEWER_LIBRARY} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTMAIN_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${OPENGL_LIBRARIES})
TARGET_LINK_LIBRARIES(alu_glviewer basics)
TARGET_LINK_LIBRARIES(alu_glviewer imageWidget)

INSTALL(TARGETS alu_glviewer
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ssa/alu_glviewer)
