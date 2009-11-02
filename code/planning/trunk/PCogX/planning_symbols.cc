#include "planning_symbols.hh"

using namespace Planning;

Predicate::Predicate(std::string&& name,
                     bool sign,
                     Arguments_Description&& arguments_Description)
    :name(std::move(name)),
     sign(sign),
     arguments_Description(std::move(arguments_Description))
{
}

bool Predicate::operator==(const Predicate& rhs) const
{
    if(rhs.sign == sign &&
       rhs.name == name  &&
       rhs.arguments_Description == arguments_Description){
        return true;
    } else {
        return false;
    }
}

/* \member{name} then \member{sign} then \member{arguments_Description}. */
bool Predicate::operator<(const Predicate& rhs) const
{
    if(name < rhs.name ){
        return true;
    } else if (rhs.name == name) {
        if(sign < rhs.sign){
            return true;
        } else if (rhs.sign == sign) {
            if(arguments_Description < rhs.arguments_Description){
                return true;
            }
        }
    }
                    
    return false;
}


std::size_t Predicate::hash_value() const
{
    std::size_t tmp = 0;

                    
    for(auto parameter = arguments_Description.begin()
            ; arguments_Description.end() != parameter
            ; parameter++)
    {
        const Typed_Variables& typed_variables = parameter->first;
        const Variables& variables = typed_variables.first;
        const Variables& types = typed_variables.second;
        
        const Constants& constants = parameter->second;

        QUERY_UNRECOVERABLE_ERROR(constants.size() && variables.size(),
                                  "Invalid specification of predicate :: "<<(*this)<<std::endl);
                
        boost::hash_combine(tmp, variables);
        boost::hash_combine(tmp, constants);
        boost::hash_combine(tmp, types);
    }
                    
    boost::hash_combine(tmp, name);
    boost::hash_combine(tmp, sign);
                    
    VERBOSER(1, (*this)<<" = "<<tmp);
    return tmp;
}


std::ostream& std::operator<<(std::ostream& o,
                              const Planning::Predicate& predicate)
{
    o<<((!predicate.sign)?"(not ":"")<<"("<<predicate.name<<" ";
    
    for(auto parameter = predicate.arguments_Description.begin()
            ; predicate.arguments_Description.end() != parameter
            ; parameter++)
    {
        const Planning::Typed_Variables& typed_variables = parameter->first;
        const Planning::Variables& variables = typed_variables.first;
        const Planning::Types& types = typed_variables.second;
        
        const Planning::Constants& constants = parameter->second;

        QUERY_WARNING(constants.size() && variables.size(), "Invalid predicate being printed, an argument spec. element has :: "<<constants.size()<<" constants and :: "<<variables.size()<<" variables.");

        for (auto variable = variables.begin()
                 ; variables.end() != variable
                 ; variable ++){
            
            o<<'?'<<*variable<<" ";
        }
        
        if(types.size() == 1){
            const Planning::Type type = types[0];
            o<<" - "<<type<<" ";
        } else {
            o<<" - (either ";

            o<<types;
            
            o<<") ";
        }

        o<<constants;
    }
    o<<")"<<((!predicate.sign)?")":"");
    
    return o;
}


Function::Function(std::string&& name,
                   bool sign,
                   Arguments_Description&& arguments_Description,
                   Range range )
    :Predicate(std::move(name),
               sign,
               std::move(arguments_Description)),
             range(range)
{
}

bool Function::operator<(const Function& rhs) const
{
    if(range < rhs.range){
        return true;
    } else if (rhs.range == range) {
        if(Predicate::operator<(rhs)){
            return true;
        }
    }

    return false;
}


bool Function::operator==(const Function& rhs) const
{
    return (rhs.range == range && Predicate::operator==(rhs));
}
        

    /* FIX :: This does not support function overloading.*/
std::size_t Planning::hash_value(const Planning::Predicate& predicate)
{return predicate.hash_value();}
