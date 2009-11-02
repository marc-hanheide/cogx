#ifndef PLANNING_SYMBOLS_HH
#define PLANNING_SYMBOLS_HH

#include "global.hh"

namespace Planning
{
    typedef std::string Type;
    typedef std::string Variable;
    typedef std::string Constant;
    typedef std::string Requirement;
            
    typedef Traversable_Container_of_Unique_Elements<Requirement> Requirements;
    typedef Traversable_Container_of_Unique_Elements<Variable> Variables;
    typedef Traversable_Container_of_Unique_Elements<Constant> Constants;
    typedef Traversable_Container_of_Unique_Elements<Type> Types;
            
    typedef std::pair<Variables, Types> Typed_Variables;
            
    typedef std::pair<Typed_Variables, Constants> Typed_Variables_Or_Constants;
            
    typedef std::vector<Typed_Variables_Or_Constants> Arguments_Description;


    class Predicate;
    class Function;
}

namespace std
{
    ostream& operator<<(ostream& o, const Planning::Predicate& predicate);
}


namespace Planning
{
    class Predicate
    {
    public:
        Predicate(std::string&& name = "",
                  bool sign = false,
                  Arguments_Description&& arguments_Description = Arguments_Description());
        
        friend std::ostream& std::operator<<(std::ostream& o, const Predicate& predicate);

        /* Name */
        std::string name;
                
        /* Sign. \value{true} means positive, \value{false} means
         * negative.*/
        bool sign;

        /* Description of the predicate arguments. */
        Arguments_Description arguments_Description;

        /* \member{sign} then \member{name} then \member{arguments_Description}*/
        bool operator==(const Predicate&) const;

        /* \member{name} then \member{sign} then \member{arguments_Description}. */
        bool operator<(const Predicate&) const;
  
        std::size_t hash_value() const;
    };
}

namespace Planning
{
    class Function: public Predicate
    {
    public:
        enum Range { s_number, s_int, s_double };
        Range range;
        
        Function(std::string&& name,
                 bool sign,
                 Arguments_Description&& arguments_Description,
                 Range range = s_number);

        /* \member{range} then (see \parent)*/
        bool operator<(const Function&) const;

        /* \member{range} and (see \parent)*/
        bool operator==(const Function&) const;
    };
    
    typedef Traversable_Container_of_Unique_Elements<Predicate> Predicates;
    
    typedef Traversable_Container_of_Unique_Elements<Function> Functions;

    /* FIX :: This does not support function overloading.*/
    std::size_t hash_value(const Planning::Predicate& predicate);//{return predicate.hash_value();};
}

#endif
