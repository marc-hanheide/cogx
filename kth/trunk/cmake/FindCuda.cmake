
find_path(CUDA_INSTALLROOT
  NAMES include/cuda.h
  PATHS
	/usr/local/cuda
	/opt/cuda
	/opt/local/cuda
	/Users/luser/packages/cuda
  DOC "The path where CUDA is installed"
)

if (CUDA_INSTALLROOT)
  set(CUDA_FOUND "YES")
endif (CUDA_INSTALLROOT)

if (NOT CUDA_INSTALLROOT)
  message("The CUDA installation was not found.")
endif (NOT CUDA_INSTALLROOT)

