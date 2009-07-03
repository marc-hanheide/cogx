

set(server_exe cast-server)
set(server_script run-cast-server.sh)

add_custom_target(${server_exe}  ALL
		   COMMAND echo "#!/bin/bash" >> ${server_exe} 
	           COMMAND echo "# Auto-generated CAST server script" > ${server_exe} 
		   COMMAND echo "${CMAKE_INSTALL_PREFIX}/bin/${server_script} ${CMAKE_INSTALL_PREFIX}" >> ${server_exe} 
		   COMMENT "generating server script"
		   WORKING_DIRECTORY ${CAST_ROOT}/bin
		   VERBATIM)


set(client_exe cast-client)
set(client_script run-cast-client.sh)

		         add_custom_target(${client_exe} ALL
		   COMMAND echo "#!/bin/bash" >> ${client_exe} 
	           COMMAND echo "# Auto-generated CAST client script" > ${client_exe} 
		   COMMAND echo "if [ $# -ne 1 ] ; then" >> ${client_exe}
		   # cmake is borken for the below
		   # COMMAND echo "   echo "Usage: `basename \$0` <cast file>" " >> ${client_exe}
		   COMMAND echo "   echo 'Usage: ${client_exe} <cast file>" ' >> ${client_exe}
		   COMMAND echo "   exit \$E_BADARGS" >> ${client_exe}	 
		   COMMAND echo "fi" >> ${client_exe}
		   COMMAND echo "" >> ${client_exe}
		   COMMAND echo "${CMAKE_INSTALL_PREFIX}/bin/${client_script} ${CMAKE_INSTALL_PREFIX} \$1" >> ${client_exe} 
		   COMMENT "generating client script"
		   WORKING_DIRECTORY ${CAST_ROOT}/bin
		   VERBATIM)

#add_dependencies(cast-server-c++ ${server_exe} ) 
#add_dependencies(cast-server-c++ ${client_exe}) 

set(exes ${server_exe} ${server_script} ${client_exe} ${client_script})
install(PROGRAMS ${exes} DESTINATION bin PERMISSIONS OWNER_READ OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
