option(BUILD_SA_CONCEPTUAL_OBJECT_RECOGNIZER_MANAGER "Build Conceptual.SA ObjectRecognizerManager component" YES)
if(BUILD_SA_CONCEPTUAL_OBJECT_RECOGNIZER_MANAGER)

    add_cast_component(ConceptualObjectRecognizerManager ObjectRecognizerManager.cc)
    link_cast_component(${CAST_COMPONENT_NAME} ConceptualData VisionData FrontierInterface)

endif(BUILD_SA_CONCEPTUAL_OBJECT_RECOGNIZER_MANAGER)
