# Find the things that cast requires, and complain if they're not there

find_package(Boost)
if(NOT Boost_FOUND)
	message( FATAL_ERROR "Boost not found. Please install it before installing CAST." )
endif(NOT Boost_FOUND)


find_package(Ice)
if(NOT ICE_FOUND)
       message( FATAL_ERROR "Ice not found. Please install it before installing CAST." )
endif(NOT ICE_FOUND)

include( UseBoost )
include( UseIce )



