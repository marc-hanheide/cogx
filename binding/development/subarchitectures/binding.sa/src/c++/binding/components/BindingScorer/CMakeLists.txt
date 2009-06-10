PROJECT(BindingScorer)

# Set the sources.
SET(SOURCES BindingScorer.cpp ComparatorCache.cpp)
SET(HEADERS BindingScorer.hpp ComparatorCache.hpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LINK_FLAGS} ${OMNIORB4_CFLAGS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkInterface FrameworkIDL FrameworkCore)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTCore CASTArchitecture CASTIDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} BindingIDL BindingSubarch)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/binding/components/BindingScorer/)
