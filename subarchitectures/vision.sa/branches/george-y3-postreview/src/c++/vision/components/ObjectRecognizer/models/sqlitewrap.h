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
#ifndef SQLITEWRAP_A3SAAA2Q
#define SQLITEWRAP_A3SAAA2Q

#include "Exception.h"
#include <sqlite3.h>
#include <string>
#include <cstring>
#include <vector>
#include <iostream>

namespace cogx { namespace vision {
struct CSqliteStatement
{
   sqlite3* __pdb;
   sqlite3_stmt* pStatement;
   char* pTail;
   bool closed;

   CSqliteStatement(sqlite3* db, const char* sql) {
      __pdb = db;
      int sqlrv = sqlite3_prepare_v2(db, sql, -1, &pStatement, (const char**) &pTail);
      if (sqlrv != SQLITE_OK) 
         throw Exception(errmsg(__pdb));
      closed = false;
   }

   ~CSqliteStatement() {
      if (pStatement && ! closed) {
         int sqlrv = sqlite3_finalize(pStatement);
         closed = true;
      }
   }

   static const std::string errmsg(sqlite3* db) {
      return std::string(sqlite3_errmsg(db));
   }

   int step() const {
      return sqlite3_step(pStatement);
   }

   void bind_int64(int field, long long val) const {
      int sqlrv = sqlite3_bind_int64(pStatement, field, val);
      if (sqlrv != SQLITE_OK) 
         throw Exception(errmsg(__pdb));
   }

   void bind_text(int field, const std::string& text) const {
      //std::cout << "binding " << field << " " << text << " " << text.size() << std::endl;
      int sqlrv = sqlite3_bind_text(pStatement, field, text.data(), text.size(), SQLITE_STATIC);
      if (sqlrv != SQLITE_OK) 
         throw Exception(errmsg(__pdb));
   }

   long long get_int64(int field) const {
      return sqlite3_column_int64(pStatement, field);
   }

   void getBlob(int field, std::vector<unsigned char>& bytes) const {
      int len = sqlite3_column_bytes(pStatement, field);
      const void* pData = sqlite3_column_blob(pStatement, field);
      bytes.resize(len);
      memcpy(&bytes[0], pData, len);
   }

   void getText(int field, std::string& text) const {
      int len = sqlite3_column_bytes(pStatement, field);
      const char* pData = (const char*)sqlite3_column_text(pStatement, field);
      text.clear();
      text.append(pData, len);
   }
};

}} // namespace
#endif /* end of include guard: SQLITEWRAP_A3SAAA2Q */
// vim:sw=3:ts=8:et
