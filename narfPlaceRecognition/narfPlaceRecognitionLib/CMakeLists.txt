#INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} )

ADD_LIBRARY(narfPlaceRecognitionLib
  rangeImageMatching.h rangeImageMatching.cpp
  scanDatabase.h scanDatabase.cpp
  sparsePointCloud.h
)
TARGET_LINK_LIBRARIES(narfPlaceRecognitionLib
  narfssKeypoint
  ${PCL_COMMON_LIBRARY} ${PCL_IO_LIBRARY} ${PCL_RANGE_IMAGE_LIBRARY} ${PCL_KEYPOINTS_LIBRARY} ${PCL_FEATURES_LIBRARY}
)
