/*
 * @author:  Marko Mahnič
 * @created: nov 2009 
 */
	
#ifndef OBJECT_ANALYZER_TYPE_MAPPER_H
#define OBJECT_ANALYZER_TYPE_MAPPER_H

#include <vector>
#include <string>
#include <map>

namespace cogx
{

typedef std::vector<std::string> TStringVector;
typedef std::vector<double> TDoubleVector;
typedef std::map<std::string,std::string>  TStringMap;

class CVisualTypeMapper
{
	TStringMap m_TypeMap;
	bool m_useTypeMap;
  public:
	CVisualTypeMapper();
	void configure(const TStringMap & _config);
	void mapLabels(
		const TStringVector &labels, const TDoubleVector &distrLabels,
	   	TStringVector &types, TDoubleVector &distrTypes);
};

} // namespace
#endif
