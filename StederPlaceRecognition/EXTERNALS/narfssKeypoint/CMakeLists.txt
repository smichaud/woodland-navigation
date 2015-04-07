INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} )

ADD_LIBRARY(narfssKeypoint
  narfss_keypoint.h narfss_keypoint.hpp narfss_keypoint.cpp
  farthestPoint.h farthestPoint.hpp
)
TARGET_LINK_LIBRARIES(narfssKeypoint
  ${PCL_KEYPOINTS_LIBRARY} ${PCL_KDTREE_LIBRARY} ${PCL_SEARCH_LIBRARY}
)
