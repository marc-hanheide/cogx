/*
 * Author: Marko Manič
 * Created: 2012-07-05
 *
 * © Copyright 2012 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "belief_utils.hpp"
#include <beliefs_cast.hpp>

namespace beliefcore = de::dfki::lt::tr::beliefs::slice;

namespace cast { namespace beliefs {

cast::cdl::WorkingMemoryPointerPtr recurseAncestorsForType(cast::ManagedComponent& component,
      beliefcore::sitbeliefs::dBeliefPtr root, const std::string& typeName)
{
  auto pmrgHist = dynamic_cast<beliefcore::history::CASTBeliefHistory*>(root->hist.get());
  if (pmrgHist) {
    //m_display.setHtml("BELIEF-TESTING", pbel->id + "b", pmrgHist->ice_id());

    // Go through all WM pointers in ancestors to find a pointer to typeName
    for (auto pwmpAncestor : pmrgHist->ancestors) {
      if (pwmpAncestor->type == typeName) {
        //component.println(" **** Found '%s' (%s) **** ",
        //    pwmpAncestor->type.c_str(),
        //    pwmpAncestor->address.id.c_str());
        return pwmpAncestor;
      }
    }

    // Go through all WM pointers and visit the ancestors
    for (auto pwmpAncestor : pmrgHist->ancestors) {
      beliefcore::sitbeliefs::dBeliefPtr pancBel;
      try {
        pancBel = component.getMemoryEntry<beliefcore::sitbeliefs::dBelief>(pwmpAncestor->address);
        if (!pancBel.get()) {
          // wm entry exists but it's not a dBelief
          //component.println(" **** '%s' (%s) is not a dBelief **** ",
          //    pwmpAncestor->type.c_str(),
          //    pwmpAncestor->address.id.c_str());
          continue;
        }

        auto pfound = recurseAncestorsForType(component, pancBel, typeName);
        if (pfound.get() != nullptr) {
          return pfound;
        }
      }
      catch(DoesNotExistOnWMException){
        continue;
      }
    }
  }
  return nullptr;
}

}} // namespace
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et :vim
