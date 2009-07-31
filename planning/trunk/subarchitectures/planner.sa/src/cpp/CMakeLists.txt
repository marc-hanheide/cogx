include_directories(.)
include_directories(../../../fakebinder.sa/src/c++)
include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP Planner.ice ${CAST_SLICE_DIR})

set(GUI_CPP_SRC MainWindow.cpp MainWidget.cpp)
set(GUI_HPP_SRC MainWindow.hpp MainWidget.hpp)

qt4_wrap_cpp(GUI_MOC_SRC ${GUI_HPP_SRC})

include_directories(${QT_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR} ${QT_QTCORE_INCLUDE_DIR})

add_library(Planner SHARED ${GEN_CPP} ${GUI_CPP_SRC} ${GUI_MOC_SRC})
target_link_libraries(Planner ${QT_QTGUI_LIBRARY} ${QT_QTCORE_LIBRARY} ${QT_QTASSISTANT_LIBRARY} FakeBinderData)
install(TARGETS Planner LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

add_cast_component(WMControl WMControl.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner)
