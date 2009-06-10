#ifndef DOT_DOT_UTIL_H_
#define DOT_DOT_UTIL_H_

#include <string>

namespace dot {

extern std::string dot_head(const std::string& _title = "");
extern std::string dot_foot();

/// removes things that are not allowed in a dot node id
extern std::string fix_dot_id(const std::string& _str);

extern std::string dot_node(const std::string& _id, 
		       const std::string& _label = "", 
		       const std::string& _shape = "box",
		       const std::string& _properties = "");

extern
std::string dot_arrow( const std::string& _from, 
		  const std::string& _to, 
		  const std::string& _type = "->", 
		  const std::string& _label = "",
		  const std::string& _properties = "");

extern
std::string boxed_dot_arrow( const std::string& _from, 
			const std::string& _to, 
			const std::string& _type = "->", 
			const std::string& _label = "",
			const std::string& _properties = "",
			const std::string& _box_type = "diamond");
} // namespace dot

#endif // DOT_DOT_UTIL_H_
