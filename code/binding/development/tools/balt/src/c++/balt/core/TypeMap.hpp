/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * 
 * Copyright (C) 2006-2007 Nick Hawes, Gregor Berginc
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 * 
 */

// HenrikJ: added this, largely based on Stroustrup

#ifndef BALT_TYPE_MAP_H_
#define BALT_TYPE_MAP_H_

#include <ext/hash_map>
#include <typeinfo>

namespace balt {
namespace detail {

struct TI_eq {
  bool operator()(const std::type_info* _p, const std::type_info* _q) const {return *_p==*_q;}
};

struct TI_hash {
  int operator()(const std::type_info* _p) const {return reinterpret_cast<int>(_p); }
};
} // namespace detail
} // namespace balt

/// see Stroustrup 15.4.4.1 (\remark, should be moved into a balt namespace later)
template<typename TypeInfoT>
struct TypeMap {
  typedef __gnu_cxx::hash_map<const std::type_info*, TypeInfoT, balt::detail::TI_hash, balt::detail::TI_eq> map;
private:
  /// not constructable
  ~TypeMap();
};

#endif // BALT_TYPE_MAP_H_

