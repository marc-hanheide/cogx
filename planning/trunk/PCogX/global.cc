#include "global.hh"

using std::ostringstream;
using std::endl;
using std::string;

bool HasStringRepresentation::operator<(const HasStringRepresentation& hsr) const
{
    /*FIX :: efficienct (i.e., repeat conditional from overloaded
     * \function{std::operator<<} below...).*/
    VERBOSER(5, "Calling LEQ test for HasStringRepresentation.\n");
    
    ostringstream oss;
    oss<<*this;
    ostringstream oss1;
    oss1<<hsr;

    VERBOSER(5, "EQ test between "
	     <<*this<<" and "<<hsr<<" yields :: "<<(oss.str() < oss1.str())<<endl);
    
    return oss.str() < oss1.str();
}

bool HasStringRepresentation::operator==(const HasStringRepresentation& hsr) const
{
    /*FIX :: efficienct (i.e., repeat conditional from overloaded
     * \function{std::operator<<} below...).*/
    VERBOSER(5, "Calling EQ test for HasStringRepresentation.\n");
    ostringstream oss;
    oss<<*this;
    ostringstream oss1;
    oss1<<hsr;

    VERBOSER(5, "EQ test between "
	     <<*this<<" and "<<hsr<<" yields :: "<<(oss.str() == oss1.str())<<endl);
    
    return oss.str() == oss1.str();
}

size_t HasStringRepresentation::hash_value() const
{
    /*FIX :: efficienct (i.e., repeat conditional from overloaded
     * \function{std::operator<<} below...).*/
    ostringstream oss;
    oss<<*this;
    
    return hasher(oss.str());
}


HasStringRepresentation::HasStringRepresentation(const string& str, bool b)
    :asString(str),computedAsString(b)
{}


HasStringRepresentation::HasStringRepresentation(string&& str, bool b)
    :asString(str),computedAsString(b)
{}

HasStringRepresentation::~HasStringRepresentation(){}
    
HasStringRepresentation::HasStringRepresentation(const HasStringRepresentation& hsr)
    :asString(hsr.asString),
     computedAsString(hsr.computedAsString)
{}

HasStringRepresentation::HasStringRepresentation(HasStringRepresentation&& hsr)
    :asString(std::move(hsr.asString)),
     computedAsString(hsr.computedAsString)
{}
    
HasStringRepresentation& HasStringRepresentation::operator=(HasStringRepresentation&& hsr)
{
    asString = std::move(hsr.asString);
    computedAsString = computedAsString;

    return *this;
}
    
HasStringRepresentation& HasStringRepresentation::operator=(const HasStringRepresentation& hsr)
{
    asString = hsr.asString;
    computedAsString = computedAsString;

    return *this;
}


std::size_t hash_value(const HasStringRepresentation& hsr)
{
    return hsr.hash_value();
}

namespace std
{
    std::ostream& operator<<(std::ostream& o,
                             const HasStringRepresentation& hasStringRepresentation)
    {
        if(!hasStringRepresentation.computedAsString){
            hasStringRepresentation.computeAsString("");
        }
        
        return o<<hasStringRepresentation.asString;
    }
}

