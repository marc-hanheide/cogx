/*
 * @author:  Marko Mahnič
 * @created: nov 2009 
 */
   
#include "Enumerator.h"
#include <iostream>

using namespace std;

namespace matlab {

//################################################################# 
CTypeEnumerator::CTypeEnumerator(bool bAllowDupNames)
{
   m_bAllowDupNames = bAllowDupNames;
}

void CTypeEnumerator::operator= (const CTypeEnumerator& typeEnum)
{
   clear();
   map<string,long>::const_iterator ps;
   for (ps = typeEnum.m_fwdMap.begin(); ps != typeEnum.m_fwdMap.end(); ps++) {
      addMapping(ps->first, ps->second);
   }
}

void CTypeEnumerator::clear()
{
   m_fwdMap.clear();
   m_revMap.clear();
}

long CTypeEnumerator::getLastEnum()
{
   map<string,long>::const_iterator ps;
   long max = -1;
   for (ps = m_fwdMap.begin(); ps != m_fwdMap.end(); ps++) {
      if ( (*ps).second > max) max = (*ps).second;
   }
   return max;
}

void CTypeEnumerator::getLabels(TStringVector &labels)
{
   map<string,long>::const_iterator ps;
   for (ps = m_fwdMap.begin(); ps != m_fwdMap.end(); ps++) {
      labels.push_back( (*ps).first);
   }
}

bool CTypeEnumerator::addMapping(string sName, long nEnum)
{
   if (nEnum < 0) return false;
   if (sName.size() < 1) return false;
   if (m_fwdMap.count(sName) > 0) {
      if (nEnum == m_fwdMap[sName]) return true;
      return false;
   }
   if (m_revMap.count(nEnum) > 0) {
      if (! m_bAllowDupNames) return false;
      m_fwdMap[sName] = nEnum;
   }
   else {
      m_fwdMap[sName] = nEnum;
      m_revMap[nEnum] = sName;
   }
   return true;
}

long CTypeEnumerator::getEnum(string sName)
{
   if (m_fwdMap.count(sName) > 0) return m_fwdMap[sName];
   return -1;
}

string CTypeEnumerator::getName(long nEnum)
{
   if (m_revMap.count(nEnum) > 0) return m_revMap[nEnum];
   return "";
}


} // namespace
