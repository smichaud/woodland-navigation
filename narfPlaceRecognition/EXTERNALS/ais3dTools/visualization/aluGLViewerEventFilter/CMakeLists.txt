INCLUDE(${QT_USE_FILE})

# ${QT_INCLUDE_DIR} does not appear to work; therefore have to
# manually rig it; also have to add QtCore directly because qglviewer
# doesn't seem to handle the Qt directories properly

QT4_WRAP_CPP(UI_SOURCES alu_glviewer_event_filter.h alu_glwidget_stereo_event_filter.h)

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${QT_QTCORE_INCLUDE_DIR}/.. ${QGLVIEWER_INCLUDE_DIR})


ADD_LIBRARY(alu_glviewer_event_filter SHARED
  alu_glviewer_event_filter.cpp
  alu_glviewer_event_filter.h
  alu_glwidget_stereo_event_filter.cpp
  alu_glwidget_stereo_event_filter.h
  ${UI_SOURCES}
)

SET_TARGET_PROPERTIES(alu_glviewer_event_filter PROPERTIES OUTPUT_NAME ${LIB_PREFIX}alu_glviewer_event_filter)

TARGET_LINK_LIBRARIES(alu_glviewer_event_filter alu_glviewer)

INSTALL(TARGETS alu_glviewer_event_filter
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

FILE(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")

INSTALL(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ssa/alu_glviewer)
