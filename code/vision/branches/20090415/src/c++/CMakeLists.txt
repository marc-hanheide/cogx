# auto generated code
add_and_include_subdirectory (cast/cdl)

# core code that is always used: basic components, data
add_and_include_subdirectory (cast/core)

# code that produces abstract classes for subarch components
add_and_include_subdirectory (cast/architecture)

# skeleton classes for testing
add_and_include_subdirectory (cast/examples)

# more in depth examples. doesn't compile with new version yet
add_and_include_subdirectory (cast/examples/comedyarch)

# server code for managing components at runtime
add_and_include_subdirectory (cast/server)

# tests, not ready for action yet
add_and_include_subdirectory (cast/testing)

# and these are the utility headers for users

set(headers cast/core.hpp cast/architecture.hpp cast/server.hpp)

install(FILES ${headers} DESTINATION include/cast)
