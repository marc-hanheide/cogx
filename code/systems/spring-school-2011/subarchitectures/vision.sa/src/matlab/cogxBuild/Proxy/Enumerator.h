/*
 * @author:  Marko Mahnič
 * @created: nov 2009 
 */
	
/*
 * NOTE: has to be linked with -lboost_regex-mt
 */

#ifndef OBJECT_ANALYZER_TYPE_MAPPER_H
#define OBJECT_ANALYZER_TYPE_MAPPER_H

#include <vector>
#include <string>
#include <map>

namespace matlab
{

typedef std::vector<std::string> TStringVector;
typedef std::vector<double> TDoubleVector;
typedef std::map<std::string,std::string>  TStringMap;

class CTypeEnumerator
{
private:
   // addMapping restricts enum values: enum >= 0; string must be nonempty
   std::map<std::string,long> m_fwdMap;
   std::map<long,std::string> m_revMap;

   // m_bAllowDupNames:
   //	false: 1 on 1 mapping
   //	true:  multiple labels can have the same ID (synonyms)
   bool m_bAllowDupNames;
public:
   CTypeEnumerator(bool bAllowDupNames=true);
   void operator= (const CTypeEnumerator& typeEnum);
   void clear();
   bool addMapping(std::string sName, long nEnum);
   long getLastEnum();
   void getLabels(TStringVector &labels);

   // returns -1 when name is not defined
   long getEnum(std::string sName);

   // returns "" when enum is not defined
   std::string getName(long nEnum);
};


} // namespace
#endif
// vim:sw=3:ts=8:et
