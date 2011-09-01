
#ifndef P_TO_STRING_HH
#define P_TO_STRING_HH


namespace P
{

template <class T>
inline std::string toString (const T& t)
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}


}

#endif
