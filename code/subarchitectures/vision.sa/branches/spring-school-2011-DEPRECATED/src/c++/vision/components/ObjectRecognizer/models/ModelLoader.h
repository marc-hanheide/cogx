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

#ifndef MODELLOADER_ZU6DHUJK
#define MODELLOADER_ZU6DHUJK

#include "ObjectModel.h"
#include "Features.h"
#include "sqlitewrap.h"
#include <string>
#include <vector>

namespace cogx { namespace vision {

class CModelLoader
{
   std::vector<unsigned char> buf;
   void loadFeature(sqlite3* db, CSqliteStatement& cmd, CSiftFeature& feature);
   void loadView(sqlite3* db, CSqliteStatement& cmd, CObjectView& view);
   void loadModel(sqlite3* db, CSqliteStatement& cmd, CObjectModel& view);
public:
   void loadFeature(sqlite3* db, long long siftid, CSiftFeature& feature);
   void loadView(sqlite3* db, long long viewid, CObjectView& view);
   void loadModelViews(sqlite3* db, long long modelid, CObjectModel& model);
   void loadModel(sqlite3* db, long long modelid, CObjectModel& model);
   void loadModel(sqlite3* db, const std::string& modelName, CObjectModel& model);
   void listModels(sqlite3* db, std::vector<std::string>& models);
};

}} // namespace
#endif /* end of include guard: MODELLOADER_ZU6DHUJK */
// vim:sw=3:ts=8:et
