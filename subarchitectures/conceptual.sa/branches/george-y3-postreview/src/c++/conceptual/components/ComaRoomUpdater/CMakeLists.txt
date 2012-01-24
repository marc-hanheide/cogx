option(BUILD_SA_CONCEPTUAL_COMA_ROOM_UPDATER "Build Conceptual.SA ComaRoomUpdater component" YES)
if(BUILD_SA_CONCEPTUAL_COMA_ROOM_UPDATER)

    add_cast_component(ConceptualComaRoomUpdater ComaRoomUpdater.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData ComaData)

endif(BUILD_SA_CONCEPTUAL_COMA_ROOM_UPDATER)

