find_program (MERCURY_BIN_MMC mmc)
mark_as_advanced (MERCURY_BIN_MMC)

if (NOT MERCURY_BIN_MMC)
	message ("The Mercury compiler was not found.")
	set (MERCURY_HOME "" CACHE PATH "Root of the Mercury installation.")
else (NOT MERCURY_BIN_MMC)
	get_filename_component (MERCURY_BIN_DIR ${MERCURY_BIN_MMC} PATH)
	get_filename_component (MERCURY_HOME_DIR ${MERCURY_BIN_DIR} PATH)
	set (MERCURY_HOME ${MERCURY_HOME_DIR} CACHE PATH "Root of the Mercury installation.")
endif (NOT MERCURY_BIN_MMC)

#set (ENV{MERCURY_HOME} ${MERCURY_HOME} PARENT_SCOPE)
