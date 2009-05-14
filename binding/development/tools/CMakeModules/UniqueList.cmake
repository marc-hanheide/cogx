	# restore from cache
	##SET(ALL_IDL ${ALL_IDL} CACHE INTERNAL "")
	# remove item if there
	##LIST(REMOVE_ITEM ALL_IDL ${idlPath})     	  
	# add again
	##LIST(APPEND ALL_IDL ${idlPath} )
	# save to cache
	##SET(ALL_IDL ${ALL_IDL} CACHE INTERNAL "")	


MACRO (EXTEND_LIST _item _list)
#MESSAGE("extend list: " ${_item})
#MESSAGE("extend list: " ${${_list}})
	# restore from cache
	SET(${_list} ${${_list}} CACHE INTERNAL "")
	# remove item if there
	LIST(REMOVE_ITEM ${_list} ${_item})     	  
	# add again
	LIST(APPEND ${_list} ${_item} )
	# save to cache
	SET(${_list} ${${_list}} CACHE INTERNAL "")	
ENDMACRO (EXTEND_LIST _item _list)

MACRO (SPACE_LIST _list _var)
  FOREACH(ITEM ${${_list}})
    SET(${_var} "${ITEM} ${${_var}}")
  ENDFOREACH(ITEM ${${_list}})
ENDMACRO (SPACE_LIST _list _var)