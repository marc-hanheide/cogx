#ifndef CAST_BINDING_UTILS_H_
#define CAST_BINDING_UTILS_H_

#include <stdexcept> 


namespace Binding {

class BindingException : public std::runtime_error {
public: 
    BindingException(const std::string& err) : std::runtime_error(err) {}
};

} // namespace Binding


#endif
