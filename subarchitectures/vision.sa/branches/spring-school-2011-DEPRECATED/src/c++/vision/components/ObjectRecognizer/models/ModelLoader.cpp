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
#include "ModelLoader.h"
#include <cstring>
#include <iostream>

namespace cogx { namespace vision {

#define sFeatureFields "id, cx, cy, scale, orientation, descriptor"
void CModelLoader::loadFeature(sqlite3* db, CSqliteStatement& cmd, CSiftFeature& feature)
{
   feature.id = sqlite3_column_int64(cmd.pStatement, 0);
   feature.x = sqlite3_column_double(cmd.pStatement, 1);
   feature.y = sqlite3_column_double(cmd.pStatement, 2);
   feature.scale = sqlite3_column_double(cmd.pStatement, 3);
   feature.orientation = sqlite3_column_double(cmd.pStatement, 4);
   cmd.getBlob(5, buf);
   int len = buf.size();
   // assert(len == CSiftFeature::NDims)
   if (len > CSiftFeature::NDims) len = CSiftFeature::NDims;
   memcpy(feature.descriptor, &buf[0], len);
}

void CModelLoader::loadFeature(sqlite3* db, long long siftid, CSiftFeature& feature)
{
   const char sFeatures[] =
      "SELECT "  sFeatureFields 
      "  FROM sifts WHERE sifts.id=?;";
   
   CSqliteStatement cmd(db, sFeatures);
   cmd.bind_int64(1, siftid);

   int sqlrv = cmd.step(); // first row
   if (sqlrv == SQLITE_ROW) {
      loadFeature(db, cmd, feature);

      sqlrv = cmd.step(); // next row
      if (sqlrv != SQLITE_DONE) {
         // TODO: There was an error or too many rows returned ... what do we do?
      }
      return;
   }
}

#define sViewFields "id, imagefile, phi, lambda, rotation"
void CModelLoader::loadView(sqlite3* db, CSqliteStatement& cmd, CObjectView& view)
{
   view.m_id = sqlite3_column_int64(cmd.pStatement, 0);
   cmd.getText(1, view.m_imagefile);
   view.m_phi = sqlite3_column_double(cmd.pStatement, 2);
   view.m_lambda = sqlite3_column_double(cmd.pStatement, 3);
   view.m_rotation = sqlite3_column_double(cmd.pStatement, 4);

   const char sFeatures[] =
      "SELECT "  sFeatureFields 
      "  FROM sifts WHERE viewid=?;";
   
   CSqliteStatement cmdF(db, sFeatures);
   cmdF.bind_int64(1, view.m_id);

   int sqlrv = cmdF.step(); // first row
   while (sqlrv == SQLITE_ROW) {
      CSiftFeature* pFeature = new CSiftFeature();
      loadFeature(db, cmdF, *pFeature);
      view.m_features.push_back(pFeature);

      sqlrv = cmdF.step(); // next row
   }

   if (sqlrv != SQLITE_DONE) {
      // TODO: There was an error ... what do we do?
   }
}

void CModelLoader::loadView(sqlite3* db, long long viewid, CObjectView& view)
{
   const char sViews[] =
      "SELECT "  sViewFields 
      "  FROM views WHERE views.id=?;";

   CSqliteStatement cmd(db, sViews);
   cmd.bind_int64(1, viewid);

   int sqlrv = cmd.step(); // first row
   if (sqlrv == SQLITE_ROW) {
      CObjectView* pView = new CObjectView();
      loadView(db, cmd, *pView);

      sqlrv = cmd.step(); // next row (should return SQLITE_DONE)
      if (sqlrv != SQLITE_DONE) {
         // TODO: There was an error or too many rows returned ... what do we do?
      }
   }
}

#define sModelFields  "id, name"
void CModelLoader::loadModel(sqlite3* db, CSqliteStatement& cmd, CObjectModel& model)
{
   model.m_id = cmd.get_int64(0);
   cmd.getText(1, model.m_name);
}

void CModelLoader::loadModelViews(sqlite3* db, long long modelid, CObjectModel& model)
{
   const char sViews[] =
      "SELECT "  sViewFields
      "  FROM views WHERE views.modelid=?;";

   CSqliteStatement cmdV(db, sViews);
   cmdV.bind_int64(1, modelid);

   int sqlrv = cmdV.step(); // first row
   while (sqlrv == SQLITE_ROW) {
      CObjectView* pView = new CObjectView();
      loadView(db, cmdV, *pView);
      model.m_views.push_back(pView);

      sqlrv = cmdV.step(); // next row
   }

   if (sqlrv != SQLITE_DONE) {
      // TODO: There was an error ... what do we do?
   }
}

void CModelLoader::loadModel(sqlite3* db, long long modelid, CObjectModel& model)
{
   const char sModel[] =
      "SELECT "  sModelFields
      "  FROM models WHERE models.id=?;";

   CSqliteStatement cmdM(db, sModel);
   cmdM.bind_int64(1, modelid);

   int sqlrv = cmdM.step(); // first row
   if (sqlrv == SQLITE_ROW) {
      loadModel(db, cmdM, model);
   }
   else {
      model.m_id = modelid;
   }

   loadModelViews(db, modelid, model);
}

void CModelLoader::loadModel(sqlite3* db, const std::string& name, CObjectModel& model)
{
   const char sModel[] =
      "SELECT "  sModelFields
      "  FROM models WHERE models.name=?;";

   CSqliteStatement cmdM(db, sModel);
   cmdM.bind_text(1, name);

   int sqlrv = cmdM.step(); // first row
   if (sqlrv == SQLITE_ROW) {
      //std::cout << "LOADING MODEL " << name << std::endl;
      loadModel(db, cmdM, model);
   }
   else {
      //std::cout << "NO MODEL " << name << std::endl;
      model.m_id = -1;
      // TODO: throw since the model does not exist
   }

   loadModelViews(db, model.m_id, model);
}

void CModelLoader::listModels(sqlite3* db, std::vector<std::string>& models)
{
   const char sModel[] =
      "SELECT "  sModelFields   " FROM models;";

   CSqliteStatement cmdM(db, sModel);

   models.clear();
   int sqlrv = cmdM.step(); // first row
   while (sqlrv == SQLITE_ROW) {
      std::string name;
      cmdM.getText(1, name);
      models.push_back(name);

      sqlrv = cmdM.step(); // next row
   }

   if (sqlrv != SQLITE_DONE) {
      // TODO: There was an error ... what do we do?
   }
}

}} // namespace
// vim:sw=3:ts=8:et
