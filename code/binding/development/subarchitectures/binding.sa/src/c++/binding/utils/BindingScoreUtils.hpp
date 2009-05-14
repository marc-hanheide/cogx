#ifndef BINDING_SCORE_UTILS_H_
#define BINDING_SCORE_UTILS_H_

#include <binding/idl/BindingData.hh>
#include <boost/logic/tribool.hpp>


#include "binding/BindingException.hpp"


#include <iostream>
#include <sstream>
#include <cassert>


namespace Binding {

BindingData::BindingScore defaultBindingScore(); 

/// at least comparable and sticky...
BindingData::BindingScore defaultThresholdScore();
  
/// returns true iff this binding score is BETTER than \p
/// _score. The 'smallest' score is therefore the winner
bool 
operator<(const BindingData::BindingScore& _score1, 
	  const BindingData::BindingScore& _score2);

bool operator>(const BindingData::BindingScore& _score1, 
	       const BindingData::BindingScore& _score2);
bool operator==(const BindingData::BindingScore& _score1, 
		const BindingData::BindingScore& _score2);
bool operator!=(const BindingData::BindingScore& _score1, 
		const BindingData::BindingScore& _score2);
std::ostream& 
operator<<(std::ostream& _out, const BindingData::BindingScore& _score);

std::string scoreToString(const BindingData::BindingScore& _score);

boost::logic::tribool tribool_cast(cast::cdl::TriBool _tri);
cast::cdl::TriBool tribool_cast(boost::logic::tribool _tri);
std::string& triboolToString(boost::logic::tribool _tri);

} // namespace Binding


  



#endif // BINDING_SCORE_UTILS_H_
