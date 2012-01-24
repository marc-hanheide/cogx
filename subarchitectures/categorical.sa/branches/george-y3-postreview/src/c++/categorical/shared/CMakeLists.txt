project(CategoricalShared)

set(SOURCES ConfigFile.cpp DataWriter.cpp LabelFile.cpp DataReader.cpp OdomScanWidget.cpp ColorMap.cpp CastTools.cpp)  # NodePlaceInfoPullSender.cpp
set(HEADERS ConfigFile.h DataWriter.h LabelFile.h DataReader.h OdomScanWidget.h ColorMap.h CastTools.h)  # NodePlaceInfoPullSender.h
set(MOC_HEADERS OdomScanWidget.h)

qt4_wrap_cpp( MOC_SOURCES ${MOC_HEADERS} )

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${MOC_SOURCES})

target_link_libraries(${PROJECT_NAME} boost_filesystem boost_system)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
target_link_libraries(${PROJECT_NAME} CDL CategoricalData)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
