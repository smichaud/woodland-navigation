CMAKE_MINIMUM_REQUIRED(VERSION 2.6)



# IF(${PROJECT_NAME}_BUILD_SUITESPARSE)
#  ExternalProject_Add(
#    OpenBLAS2
#    URL http://github.com/xianyi/OpenBLAS/tarball/master/xianyi-OpenBLAS-v0.2.1-0-g551f478.tar.gz
#    DOWNLOAD_DIR ${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/
#    SOURCE_DIR ${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/OpenBLAS2
#    PATCH_COMMAND cp ${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/patches/Makefile.rule ${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/OpenBLAS2/
#    CMAKE_COMMAND true
#    BUILD_IN_SOURCE 1
#    BUILD_COMMAND make && make netlib shared
#    INSTALL_COMMAND cp ${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/OpenBLAS2/libopenblas_penrynp-r1.13.so ${CMAKE_BINARY_DIR}/lib/libopenblas.so
#  )
#  SET(GOTOBLAS_LIBRARY ${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/OpenBLAS2/OpenBLAS2)
# 
#   ExternalProject_Add(
#     SuiteSparse
#     #URL http://www.cise.ufl.edu/research/sparse/SuiteSparse/SuiteSparse-3.6.1.tar.gz
#     URL http://www.cise.ufl.edu/research/sparse/SuiteSparse/SuiteSparse-4.0.2.tar.gz
#     DOWNLOAD_DIR ${CMAKE_BINARY_DIR}/EXTERNALS/
#     SOURCE_DIR ${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/SuiteSparse
#     PATCH_COMMAND cp ${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/patches/SuiteSparse_config.mk ${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/SuiteSparse/SuiteSparse_config/
#     CMAKE_COMMAND true
#     BUILD_IN_SOURCE 1
#     BUILD_COMMAND ssa_SOURCE_DIR=${${PROJECT_NAME}_SOURCE_DIR} make library
#     INSTALL_COMMAND INSTALL_DIR=${CMAKE_BINARY_DIR} make install
#     DEPENDS OpenBLAS2
#   )
#   SET(SUITE_SPARSE_ROOT "${CMAKE_BINARY_DIR}")
# ENDIF(${PROJECT_NAME}_BUILD_SUITESPARSE)

IF(${PROJECT_NAME}_BUILD_G2O)
  set(G2O_INSTALL_PREFIX "${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/g2o")
  MESSAGE("installing g2o in folder " ${G2O_INSTALL_PREFIX})
  ExternalProject_Add(
    g2o
    GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
    PREFIX ${G2O_INSTALL_PREFIX}
    INSTALL_DIR ${${PROJECT_NAME}_SOURCE_DIR}
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR} -DG2O_USE_OPENMP=${${PROJECT_NAME}_USE_OPENMP}
    #BUILD_IN_SOURCE 1
  )
  include_directories(${${PROJECT_NAME}_SOURCE_DIR}/build/include)
ENDIF(${PROJECT_NAME}_BUILD_G2O)

IF(${PROJECT_NAME}_BUILD_SSA)
  set(SSA_INSTALL_PREFIX "${${PROJECT_NAME}_SOURCE_DIR}/EXTERNALS/ssa")
  MESSAGE("installing ssa in folder " ${SSA_INSTALL_PREFIX})
  
  ExternalProject_Add(
    ssa
    GIT_REPOSITORY https://github.com/MichaelRuhnke/ssa.git
    PREFIX ${SSA_INSTALL_PREFIX}
    INSTALL_DIR ${${PROJECT_NAME}_SOURCE_DIR}  
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR} -DSSA_USE_G2O_GIT=OFF -DG2O_ROOT=${G2O_INSTALL_PREFIX} -DSSA_USE_OPENMP=${${PROJECT_NAME}_USE_OPENMP} -DPCL_DIR=${PCL_DIR}
    DEPENDS g2o
    #BUILD_IN_SOURCE 1
  )
  include_directories(${SSA_INSTALL_PREFIX}/include)
ENDIF(${PROJECT_NAME}_BUILD_SSA)
