if(ROCS_BUILD_THIRDPARTY_BOOST_PROPERTY_TREE)
	install(DIRECTORY boost/property_tree DESTINATION ${ROCS_DIR}/include/boost
		PATTERN ".svn" EXCLUDE
		PATTERN "CVS" EXCLUDE)
	# Make available to others
	include_directories(.)
endif(ROCS_BUILD_THIRDPARTY_BOOST_PROPERTY_TREE)

