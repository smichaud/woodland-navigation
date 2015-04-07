INCLUDE(${QT_USE_FILE})

# ${QT_INCLUDE_DIR} does not appear to work; therefore have to
# manually rig it; also have to add QtCore directly because qglviewer
# doesn't seem to handle the Qt directories properly

QT4_WRAP_CPP(UI_SOURCES alu_glwidget.h)

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR}/.. ${QGLVIEWER_INCLUDE_DIR})

ADD_LIBRARY(alu_glwidget 
  alu_glwidget.cpp
  alu_glwidget.h
  ${UI_SOURCES}
)

SET_TARGET_PROPERTIES(alu_glwidget PROPERTIES OUTPUT_NAME ${LIB_PREFIX}alu_glwidget)

TARGET_LINK_LIBRARIES(alu_glwidget alu_glviewer alu_glviewer_event_filter )

INSTALL(TARGETS alu_glwidget
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ssa/alu_glwidget)
