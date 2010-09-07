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

#ifndef OBJECTMODEL_NKBUMPY0
#define OBJECTMODEL_NKBUMPY0

#include "sifts/Features.h"

#include <string>
#include <vector>

namespace cogx { namespace vision {

class CObjectView
{
   friend class CModelLoader;
public:
   CObjectView();
   long long m_id;
   std::string m_imagefile; // image filename without path
   // TODO: views might be missing pose info -- need to mark (flag or impossible value)
   float m_phi, m_lambda, m_rotation;
   TSiftVector m_features;
};

class CObjectModel
{
   friend class CModelLoader;
public:
   ~CObjectModel();
   long long m_id;
   std::string m_name;
   std::vector<CObjectView*> m_views; // views are owned by the model

   void getAllFeatures(std::vector<TSiftVector*>& features);
};

}} //namespace
#endif /* end of include guard: OBJECTMODEL_NKBUMPY0 */
// vim:sw=3:ts=8:et
