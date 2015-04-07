ADD_LIBRARY(spm_dictionary_selection
  spmDictionarySelection.cpp  
  spmDictionarySelection.h  
)

SET_TARGET_PROPERTIES(basics PROPERTIES OUTPUT_NAME ${LIB_PREFIX}spm_dictionary_selection)


INSTALL(TARGETS spm_dictionary_selection
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

#ADD_EXECUTABLE(spmDictionarySelector
  #spmDictionarySelector.cpp
#)
#TARGET_LINK_LIBRARIES(spmDictionarySelector spm_dictionary_selection spm_base kmeans++ ${Boost_SYSTEM_LIBRARY} ${PCL_IO_LIBRARY} ${PCL_COMMON_LIBRARY} ${PCL_KDTREE_LIBRARY} ${PCL_FEATURES_LIBRARY})
