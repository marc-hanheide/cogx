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
#ifndef _CAST_BELIEF_UTILS_HPP_4FF55851_
#define _CAST_BELIEF_UTILS_HPP_4FF55851_

#include <beliefs.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include <string>

namespace cast { namespace beliefs {

extern cast::cdl::WorkingMemoryPointerPtr recurseAncestorsForType(
    cast::ManagedComponent& component,
    de::dfki::lt::tr::beliefs::slice::sitbeliefs::dBeliefPtr root,
    const std::string& typeName);

}} // namespace
#endif /* _CAST_BELIEF_UTILS_HPP_4FF55851_ */
// vim: set fileencoding=utf-8 sw=2 sts=4 ts=8 et :vim
