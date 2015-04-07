ADD_LIBRARY(spm_base
  likelihoodModels.cpp
  likelihoodModels.h
  narfImport.cpp
  narfImport.h
  narfInstance.h
  narfWord.cpp
  narfWord.h
  spModel.cpp
  spModel.h
)

SET_TARGET_PROPERTIES(basics PROPERTIES OUTPUT_NAME ${LIB_PREFIX}spm_base)


INSTALL(TARGETS spm_base
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
