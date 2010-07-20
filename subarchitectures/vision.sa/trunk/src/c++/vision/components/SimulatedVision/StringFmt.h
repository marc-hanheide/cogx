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
         size_t beg = pos;
         if (ch == '-') { ss.setf(std::ios::left); pos++; }
         else if (ch == '+') { ss.setf(std::ios::internal); pos++; }
         size_t end = fmt.find(':', pos);
         if (end < 0) end = len;
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
               if (n > 0) ss.width(n);
               n = 0;
            }
         }
         if (n > 0) {
            if (wp == 0) ss.width(n);
            else if (wp == 1) ss.precision(n);
         }
         pos = end-1;
      }

      pos++;
   }
   ss << i;
   return ss.str();
}

#endif /* end of include guard: STRINGFMT_DO03FM55 */
// vim:sw=3:ts=8:et
