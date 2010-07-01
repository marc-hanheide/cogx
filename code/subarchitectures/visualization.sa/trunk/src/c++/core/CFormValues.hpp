/*
 * Author: Marko Mahnič
 * Created: 2010-07-01
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
#ifndef CFORMVALUES_QQ2XCJUG
#define CFORMVALUES_QQ2XCJUG

#include <string>
#include <sstream>
#include <cctype>
#include <map>
#include <stdexcept>

namespace cogx { namespace display {

// A utility class for managing form data on the client side.
class CFormValues
{
public:
   class field
   {
   public:
      std::string name;
      std::string value;
      field(const std::string& fieldname="") {
         name = fieldname;
      }
      virtual void clear() {
         value = "";
      }
      virtual void setValue(const std::string& _value) {
         value = _value;
      }
      virtual void setValue(const std::string& xpath, const std::string& _value) {
         assert(false);
      }
      virtual std::string get(const std::string& xpath) {
         return value;
      }
      virtual void get(std::map<std::string, std::string>& fieldmap) {
         fieldmap[name] = value;
      }
      virtual void dump(std::vector<std::string>& dump) {
         dump.push_back(name + "=" + value);
      }
   };
   class valuelist {
   public:
      std::vector<std::string> items;
      virtual valuelist& operator << (const std::string& value) {
         items.push_back(value);
         return *this;
      }
   };
   class set: public field
   {
   public:
      std::map<std::string, bool> items;
      set(const std::string& name, valuelist& setitems): field(name) {
         typeof(setitems.items.begin()) it;
         for(it = setitems.items.begin(); it != setitems.items.end(); it++) {
            items[*it] = false;
         }
      }
      void clear() {
         typeof(items.begin()) it;
         for(it = items.begin(); it != items.end(); it++) {
            it->second = false;
         }
      }
      virtual void dump(std::vector<std::string>& dump) {
         typeof(items.begin()) it;
         for(it = items.begin(); it != items.end(); it++) {
            dump.push_back(it->first + "=" + (it->second ? "true" : "false"));
         }
      }
      virtual std::string get(const std::string& xpath) {
         typeof(items.begin()) it = items.find(xpath);
         if (it == items.end()) return ""; // throw... wrong xpath
         else return it->second ? "1" : "0";
      }
      virtual void get(std::map<std::string, std::string>& fieldmap) {
         std::ostringstream ss;
         bool first = true;
         typeof(items.begin()) it;
         for(it = items.begin(); it != items.end(); it++) {
            if (it->second) {
               if (first) ss << it->first;
               else {
                  ss << '\n' << it->first;
                  first = false;
               }
            }
         }
         fieldmap[name] = ss.str();
      }
      virtual void setValue(const std::string& _value) {
         std::stringstream ss(_value);
         std::string item;
         while(std::getline(ss, item, '\n')) {
            typeof(items.begin()) it = items.find(item);
            if (it != items.end()) it->second = true;
         }
      }
      virtual void setValue(const std::string& xpath, const std::string& _value) {
         typeof(items.begin()) it = items.find(xpath);
         if (it == items.end()) return; // throw... wrong xpath
         else {
            std::string tmp;
            tmp.reserve(_value.size());
            std::transform(_value.begin(), _value.end(), tmp.begin(), tolower);
            it->second = (tmp=="true" || tmp=="on" || tmp=="yes" || tmp=="1") ? true : false;
         }
      }
   };
   // A set in which at most one element can be chosen
   class choice: public set
   {
   public:
      choice(const std::string& name, valuelist& setitems): set(name, setitems) {}
   };

   // TODO: add destructor!!
   std::map<std::string, field*> fields;
   void clear() {
      typeof(fields.begin()) it;
      for(it = fields.begin(); it != fields.end(); it++) {
         it->second->clear();
      }
   }
   void apply(const std::map<std::string, std::string>& newfields) {
      clear();
      typeof(newfields.begin()) itnew;
      for(itnew = newfields.begin(); itnew != newfields.end(); itnew++) {
         typeof(fields.begin()) it = fields.find(itnew->first);
         if (it != fields.end()) {
            it->second->setValue(itnew->second);
         }
      }
   }
   void setValue(const std::string& xpath, const std::string& value) {
      std::string main, sub;
      size_t pos = xpath.find('/');
      if (pos == std::string::npos) main = xpath;
      else {
         main = xpath.substr(0, pos-1);
         sub = xpath.substr(pos+1);
      }
      typeof(fields.begin()) it = fields.find(main);
      if (it != fields.end()) {
         if (sub.size() == 0) it->second->setValue(value);
         else it->second->setValue(sub, value);
      } // else throw ...
   }
   void clearValue(const std::string& name) {
      typeof(fields.begin()) it = fields.find(name);
      if (it != fields.end()) {
         it->second->clear();
      } // else throw ...
   }
   std::string get(const std::string& xpath) {
      std::string main, sub;
      size_t pos = xpath.find('/');
      if (pos == std::string::npos) main = xpath;
      else {
         main = xpath.substr(0, pos-1);
         sub = xpath.substr(pos+1);
      }
      typeof(fields.begin()) it = fields.find(main);
      if (it != fields.end()) {
         return it->second->get(sub);
      }
      return "";
   }
   void get(std::map<std::string, std::string>& fieldmap) {
      fieldmap.clear();
      typeof(fields.begin()) it;
      for(it = fields.begin(); it != fields.end(); it++) {
         it->second->get(fieldmap);
      }
   }
   void add(field* pfield) {
      fields[pfield->name] = pfield;
   }
   void dump(std::vector<std::string>& dump) {
      typeof(fields.begin()) it;
      for(it = fields.begin(); it != fields.end(); it++) {
         it->second->dump(dump);
      }
   }
};

}} // namespace
#endif /* end of include guard: CFORMVALUES_QQ2XCJUG */
// vim:sw=3:ts=8:et
