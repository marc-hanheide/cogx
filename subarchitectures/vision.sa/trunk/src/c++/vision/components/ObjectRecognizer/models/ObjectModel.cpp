/*
 * @author:  Marko Mahnič
 * @created: jun 2010 
 *
 * © Copyright 2010 Marko Mahnič. 
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
#include "ObjectModel.h"

namespace cogx { namespace vision {

CObjectModel::~CObjectModel()
{
   typeof(m_views.begin()) itv;
   for(itv = m_views.begin(); itv != m_views.end(); itv++) {
      CObjectView* pView = *itv;
      if (pView) delete pView;
   }
}

}} // namespace
// vim:sw=3:ts=8:et
