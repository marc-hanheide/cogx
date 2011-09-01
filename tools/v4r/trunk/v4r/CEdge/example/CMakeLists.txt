SET(SOURCES
    cedge.cc)

ADD_EXECUTABLE(cedge ${SOURCES})
target_link_libraries(cedge ${OpenCV_LIBS} CEdge)

# -- cedge demo -------------------------------------------
SET(SOURCES
    msld.cc)

ADD_EXECUTABLE(msdl ${SOURCES})
target_link_libraries(msdl ${OpenCV_LIBS} CEdge)

# -- segm demo -------------------------------------------
SET(SOURCES
    segm.cc)

ADD_EXECUTABLE(segm ${SOURCES})
target_link_libraries(segm ${OpenCV_LIBS} CEdge)
