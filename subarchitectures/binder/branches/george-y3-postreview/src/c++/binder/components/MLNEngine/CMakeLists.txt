include_directories(.)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/cpp/autogen)
include_directories(${COGX_ROOT}/subarchitectures/binder.sa/src/c++/binder/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${VISUALIZATION_INCLUDE_DIRS})

SET(ALCHEMY_ROOT ${COGX_ROOT}/subarchitectures/binder.sa/src/c++/alchemy)
SET(ALCHEMY_SRC ${ALCHEMY_ROOT}/src)

SET(ONLINE ${ALCHEMY_SRC}/online)
SET(INFER ${ALCHEMY_SRC}/infer)
SET(LOGIC ${ALCHEMY_SRC}/logic)
SET(PARSER ${ALCHEMY_SRC}/parser)
SET(RRF ${ALCHEMY_SRC}/rrf)
SET(UTIL ${ALCHEMY_SRC}/util)
SET(LEARNWTS ${ALCHEMY_SRC}/learnwts)

include_directories(${ONLINE})
include_directories(${INFER})
include_directories(${LOGIC})
include_directories(${PARSER})
include_directories(${RRF})
include_directories(${UTIL})
include_directories(${LEARNWTS})

add_definitions(-DFEAT_VISUALIZATION_OPENCV)
#link_directories(${ALCHEMY_ROOT}/bin/obj)

# Set the sources.
SET(SOURCES MLNEngine.cpp)
SET(ALCHEMY_SOURCES  ${UTIL}/arguments.cpp ${UTIL}/powerset.cpp ${UTIL}/random.cpp
	${PARSER}/fol.cpp ${PARSER}/listobj.cpp
	${LOGIC}/predicatetemplate.cpp ${LOGIC}/functiontemplate.cpp
	${LOGIC}/function.cpp ${LOGIC}/predicate.cpp ${LOGIC}/term.cpp
	${LOGIC}/domain.cpp  ${LOGIC}/clause.cpp ${LOGIC}/clausesampler.cpp
	${INFER}/groundclause.cpp
	${LOGIC}/Polynomial.cpp
	${LOGIC}/logichelper.cpp
	${INFER}/inferutil.cpp
	${INFER}/createhypercube.cpp
	${INFER}/createhypercubebasic.cpp ${INFER}/createhypercubedt.cpp
	${INFER}/joinhypercube.cpp
	${INFER}/refinehypercube.cpp
	${INFER}/superpred.cpp ${INFER}/superclause.cpp
	${INFER}/node.cpp ${INFER}/factor.cpp)

#add_custom_command(
#    OUTPUT ${ALCHEMY_ROOT}/bin/obj
#    COMMAND make
#    WORKING_DIRECTORY ${ALCHEMY_SRC}
#    )
    
#add_custom_target(
#   alchemy
#   DEPENDS ${ALCHEMY_ROOT}/bin/obj
#    )

add_cast_component(MLNEngine ${SOURCES} ${ALCHEMY_SOURCES})

add_dependencies(MLNEngine alchemy) 
link_cast_component(${CAST_COMPONENT_NAME} BeliefsCAST Binder ${VISUALIZATION_LIBRARIES})



