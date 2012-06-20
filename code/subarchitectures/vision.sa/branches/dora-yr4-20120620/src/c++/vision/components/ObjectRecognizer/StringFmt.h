/*
 * Author: Marko Mahnič
 * Created: July 2010
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef STRINGFMT_DO03FM55
#define STRINGFMT_DO03FM55

#include <cctype>
#include <cstring>
#include <string>
#include <sstream>
#include <ostream>
#include <algorithm>

struct _s_
{
   // return first nonwhite
   static const char* lskipwhite(const char* p)
   {
      while(isspace(*p)) p++;
      return p;
   }

   // return last nonwhite
   static const char* rskipwhite(const char* p, const char* pend)
   {
      if (*pend == '\0') pend--;
      while (pend >= p && isspace(*pend)) pend--;
      return pend;
   }

   // return last nonwhite
   static const char* rskipwhite(const char* p)
   {
      const char* pend = strchr(p, '\0') - 1;
      while (pend >= p && isspace(*pend)) pend--;
      return pend;
   }

   static std::string lstrip(const char* p)
   {
      return std::string(lskipwhite(p));
   }
   static std::string lstrip(const std::string& str, const std::string& chars="\n\r\t ")
   {
      size_t ib = str.find_first_not_of(chars);
      if (ib == std::string::npos) return "";
      return str.substr(ib);
   }

   static std::string rstrip(const char* p)
   {
      const char* pend = rskipwhite(p); 
      if (pend < p) return "";
      return std::string(p, pend+1);
   }
   static std::string rstrip(const std::string& str, const std::string& chars="\n\r\t ")
   {
      size_t ie = str.find_last_not_of(chars);
      if (ie == std::string::npos) return "";
      return str.substr(0, ie+1);
   }

   static std::string strip(const char* p)
   {
      const char* beg = lskipwhite(p);
      const char* end = rskipwhite(beg); 
      if (end < beg) return "";
      return std::string(beg, end+1);
   }
   static std::string strip(const std::string& str, const std::string& chars="\n\r\t ")
   {
      size_t ib = str.find_first_not_of(chars);
      if (ib == std::string::npos) return "";
      size_t ie = str.find_last_not_of(chars);
      return str.substr(ib, ie-ib+1);
   }

   static std::string lower(const std::string& s)
   {
      std::string r;
      r.resize(s.size());
      std::transform(s.begin(), s.end(), r.begin(), ::tolower);
      return r;
   }

   static std::string upper(const std::string& s)
   {
      std::string r;
      r.resize(s.size());
      std::transform(s.begin(), s.end(), r.begin(), ::toupper);
      return r;
   }

   static std::vector<std::string> split(const std::string& s, const std::string& delim,
         int maxsplit=0, bool keepempty=true)
   {
      std::vector<std::string> res;
      size_t dlen = delim.size();
      if (dlen < 1) {
         res.push_back(s);
         return res;
      }

      int nsplit = 0;
      size_t len = s.size();
      size_t pos = 0;
      size_t tok = s.find(delim, pos);
      while (tok != s.npos) {
         if (tok == pos) {
            if (keepempty) res.push_back("");
         }
         else {
            res.push_back(s.substr(pos, tok-pos));
         }
         //std::cout << " *** " << nsplit << " " << res.back() << std::endl;
         pos = tok+dlen;
         if (maxsplit > 0) {
            nsplit++;
            if (nsplit >= maxsplit) break;
         }
         tok = s.find(delim, pos);
         //std::cout << "pos=" << pos << " tok=" << tok << std::endl;
      }
      if (pos < len) {
         res.push_back(s.substr(pos, len-pos));
         //std::cout << " *** " << nsplit << " " << res.back() << std::endl;
      }
      return res;
   }

   static void replace(std::string &str, const std::string &what,
         const std::string &newval, int maxreplace=0)
   {
      size_t findlen = what.length();
      size_t replen = newval.length();
      size_t pos = 0;
      int count = 0;
      while((pos = str.find(what, pos)) != std::string::npos) {
         str.replace(pos, findlen, newval);
         pos += replen;
         if (maxreplace > 0) {
            count++;
            if (count >= maxreplace) break;
         }
      }
   }

   static void rreplace(std::string &str, const std::string &what,
         const std::string &newval, int maxreplace=0)
   {
      size_t findlen = what.length();
      size_t replen = newval.length();
      size_t pos = std::string::npos;
      int count = 0;
      while((pos = str.rfind(what, pos)) != std::string::npos) {
         str.replace(pos, findlen, newval);
         pos--;
         if (maxreplace > 0) {
            count++;
            if (count >= maxreplace) break;
         }
      }
   }

   static bool startswith(const std::string& s, const std::string& start)
   {
      if (start.size() == 0) return true;
      if (start.size() > s.size()) return false;
      return (s.compare(0, start.size(), start) == 0);
   }

   static bool endswith(const std::string& s, const std::string& end)
   {
      if (end.size() == 0) return true;
      if (end.size() > s.size()) return false;
      size_t p = s.size() - end.size();
      return (p == s.find(end, p));
   }
};

template<typename T>
std::string _str_(T i, short width, unsigned short precision, char fill=' ')
{
   std::ostringstream ss;
   if (width != 0) {
      if (width < 0) {
         width = -width;
         ss.setf(std::ios::left);
      }
      ss.width(width);
      ss.fill(fill);
   }
   ss.precision(precision);
   ss << i;
   return ss.str();
}

template<typename T>
std::string _str_(T i, short width=0, char fill=' ')
{
   return _str_(i, width, 6, fill);
}

// "", "d"  - generic representation
// "x" - hex representation
// "o" - oct representation
// "f" - fixed representation
// "s" - scientific representation
// ":21" - width 21, right aligned
// ":-4" - width 4, left aligned
// ":+8" - width 8, justified
// ":11.3" - width 11, precision 3
// "s:10: " - scientific, width 10, fill with spaces
template<typename T>
std::string _str_(T i, const std::string& fmt)
{
   enum _stage_ { stype = 0, spos = 1, sfill = 2 };
   std::ostringstream ss;
   int stage = stype;
   size_t len = fmt.size();
   size_t pos = 0;
   int width = -1, precision = -1;
   while(pos < len) {
      char ch = fmt[pos];
      if (stage == sfill) {
         ss.fill(ch);
         break;
      }
      if (ch == ':') {
         stage++;
         pos++;
         continue;
      }
      if (stage == stype) {
         switch (ch) {
            case 'a': ss.setf(std::ios::boolalpha); break;
            case 'b': ss.setf(std::ios::showbase); break;
            case 'd': ss.setf(std::ios::dec, std::ios::basefield); break;
            case 'o': ss.setf(std::ios::oct, std::ios::basefield); break;
            case 'x': ss.setf(std::ios::hex, std::ios::basefield); break;
            case 's': ss.setf(std::ios::scientific, std::ios::floatfield); break;
            case 'f': ss.setf(std::ios::fixed, std::ios::floatfield); break;
            default: break;
         }
      }
      else if (stage == spos) {
         if (ch == '-') { ss.setf(std::ios::left); pos++; }
         else if (ch == '+') { ss.setf(std::ios::internal); pos++; }
         size_t end = fmt.find(':', pos);
         if (end == fmt.npos) end = len;
         int n = 0;
         int wp = 0;
         while (pos < end) {
            ch = fmt[pos];
            if (ch >= '0' && ch <= '9') {
               n = n * 10 + ch - '0';
               pos++;
               continue;
            }
            if (ch != '.') { // error, invalid char
               n = 0;
               break;
            }
            if (wp > 0) break; // error, second point
            else {
               wp++;
               if (n > 0) width = n;
               n = 0;
               pos++;
            }
         }
         if (n > 0) {
            if (wp == 0) width = n;
            else if (wp == 1) precision = n;
         }
         pos = end-1;
      }

      pos++;
   }
   if (width >= 0) ss.width(width);
   if (precision >= 0) ss.precision(precision);
   // std::cout << "width=" << width << " precision=" << precision << std::endl;
   ss << i;
   return ss.str();
}

#endif /* end of include guard: STRINGFMT_DO03FM55 */
// vim:sw=3:ts=8:et
