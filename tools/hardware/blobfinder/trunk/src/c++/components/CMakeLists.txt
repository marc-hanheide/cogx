include_directories(..)
include_directories(.)




option(BUILD_HAL_BLOBFINDER_PLAYER "Build hardware ptz: player version" YES)
if(BUILD_HAL_BLOBFINDER_PLAYER)
  find_package(Player REQUIRED)
  add_cast_component(PlayerBlobFinderServer PlayerBlobFinderServer.cpp PlayerBlobFinderServer.hpp)
  link_cast_component(${CAST_COMPONENT_NAME} ${PLAYER_LIBRARIES} BlobFinder)
endif(BUILD_HAL_BLOBFINDER_PLAYER)
