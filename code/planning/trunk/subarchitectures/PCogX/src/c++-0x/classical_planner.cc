#include "classical_planner.hh"


extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new Classical_Planner();
  }
}

Classical_Planner::Classical_Planner(Designator&& name)
    :Implement(name)
{   
}


void ::implement__distinctPlanner(PCogX::distinctPlannerPtr& input){
    VERBOSER(200, "");

    Implement::add_designator(input->additionalDesignationIsAnArgument);
}

void HelloWriter::implement__readPropositionIdentifiers(PCogX::readPropositionIdentifiersPtr& input){
    VERBOSER(200, "");
}
void HelloWriter::implement__postSTRIPSAction(PCogX::postSTRIPSActionPtr& input){
    VERBOSER(200, "");
}
void HelloWriter::implement__postActionDefinition(PCogX::postActionDefinitionPtr& input){
    VERBOSER(200, "");
}
void HelloWriter::implement__postTypes(PCogX::postTypesPtr& input){
    VERBOSER(200, "");
}
void HelloWriter::implement__postXXsubtypeofYY(PCogX::postXXsubtypeofYYPtr& input){
    VERBOSER(200, "");
}
