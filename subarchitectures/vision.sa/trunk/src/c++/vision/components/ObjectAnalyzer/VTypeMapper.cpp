/*
 * @author:  Marko Mahnič
 * @created: nov 2009 
 */
   
#include <iostream>
#include <boost/regex.hpp>
#include "VTypeMapper.h"

namespace cogx {

using namespace std;

CVisualTypeMapper::CVisualTypeMapper()
{
   m_useTypeMap = true;
}

void CVisualTypeMapper::configure(const TStringMap & _config)
{
   TStringMap::const_iterator it;

   // map labels to types: --typemap "label:type label:type ..."
   if((it = _config.find("--typemap")) != _config.end()) {
      boost::regex rxpair ("(\\<\\w+):(\\w+\\>)");
      boost::smatch res;
      boost::match_flag_type flags = boost::match_default;
      string::const_iterator ps, pe;
      string opt = it->second;
      ps = opt.begin();
      pe = opt.end();
      long count = 0;
      while (boost::regex_search(ps, pe, res, rxpair, flags)) {
         count++;
         string lab = res[1];
         string typ = res[2];
         m_TypeMap[lab] = typ;
         cout << "typemap: " << lab << ":" << typ << endl; // TODO: logging: dump labels to log
         ps = res[0].second;
         if (ps != pe) ps++;
      }
      if (count < 1) {
         // TODO: logging: no labels defined or error in definition
         cout << "Error in --typemap: '" << opt << "'" << endl;
      }
   }

   // TOMAYBE: --typemapmode "copy" | "merge"
   // --usetypemap "yes"
   if((it = _config.find("--usetypemap")) != _config.end()) {
      string opt = it->second;
      if (opt == "yes") m_useTypeMap = true;
      else if (opt == "no") m_useTypeMap = false;
      else if (opt == "0") m_useTypeMap = false;
      else m_useTypeMap = true;
   }
   // for (int i = 0; i < 100; i++) cout << "config parsed" << endl;
}

void CVisualTypeMapper::mapLabels(
      const TStringVector &labels, const TDoubleVector &distrLabels,
      TStringVector &types, TDoubleVector &distrTypes)
{
   types.clear();
   distrTypes.clear();
   if (! m_useTypeMap) return;
   for (int i = 0; i < labels.size(); i++) {
      if (i >= distrLabels.size()) break;
      string lab = labels[i];
      if (m_TypeMap.count(lab) > 0) {
         types.push_back(m_TypeMap[lab]);
         distrTypes.push_back(distrLabels[i]);
      }
   }
}

} // namespace
