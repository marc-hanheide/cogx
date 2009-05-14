#ifndef MOTIVATION_COMPETENCE_REGISTRATION_HPP
#define MOTIVATION_COMPETENCE_REGISTRATION_HPP

#include <cast/architecture/ManagedProcess.hpp>
#include <cast/core/CASTUtils.hpp>
#include <motivation/idl/MotivationData.hh>

template <class FeatureT>
void registerFeatureGenerationCompetence(cast::ManagedProcess & _component) 
  throw (cast::AlreadyExistsOnWMException, cast::SubarchitectureProcessException)  {
  const std::string & type(cast::typeName<FeatureT>());
  motivation::idl::FeatureGenerationCompetence * reg = new   motivation::idl::FeatureGenerationCompetence();
  reg->m_isRelationLabel = false;
  reg->m_type = CORBA::string_dup(type.c_str());
  reg->m_component = CORBA::string_dup(_component.getProcessIdentifier().c_str());
  reg->m_subarchitecture = CORBA::string_dup( _component.getSubarchitectureID().c_str());
  _component.addToWorkingMemory(_component.newDataID(), reg);
  _component.log("registered feature competence: " + type);
}


#endif
