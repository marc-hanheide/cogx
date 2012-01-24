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
#include "CFormValues.hpp"
#include "Exception.h"

#include <cstdlib>
#include <cctype>
#include <sstream>
#include <cstring>
#include <algorithm>
#include <iostream>

namespace cogx { namespace display {

CFormValues::field::field(const std::string& fieldname)
{
   name = fieldname;
}

void CFormValues::field::clear()
{
   value = "";
}

void CFormValues::field::setValue(const std::string& _value)
{
   value = _value;
}

void CFormValues::field::setValue(const std::string& xpath, const std::string& _value)
{
   if (xpath.size() == 0) value = _value;
}

std::string CFormValues::field::get(const std::string& xpath)
{
   return value;
}

double CFormValues::field::getFloat(const std::string& xpath)
{
   return atof(value.c_str());
}

long CFormValues::field::getInt(const std::string& xpath)
{
   return atoi(value.c_str());
}

void CFormValues::field::get(std::map<std::string, std::string>& fieldmap)
{
   fieldmap[name] = value;
}

void CFormValues::field::dump(std::vector<std::string>& dump)
{
   dump.push_back(name + "=" + value);
}

void CFormValues::field::setState(const std::string& option, bool on)
{
   throw Exception("setState not applicable to CFormValues::field");
}

void CFormValues::field::setState(const std::string& option, const std::string& _value)
{
   throw Exception("setState not applicable to CFormValues::field");
}

bool CFormValues::field::getState(const std::string& option)
{
   throw Exception("getState not applicable to CFormValues::field");
   return false;
}

CFormValues::set::set(const std::string& name, const valuelist& setitems)
   : field(name)
{
   typeof(setitems.items.begin()) it;
   for(it = setitems.items.begin(); it != setitems.items.end(); it++) {
      items[*it] = false;
   }
}

void CFormValues::set::clear()
{
   typeof(items.begin()) it;
   for(it = items.begin(); it != items.end(); it++) {
      it->second = false;
   }
}

void CFormValues::set::dump(std::vector<std::string>& dump)
{
   dump.push_back(name + "=[" + get("") + "]");
   typeof(items.begin()) it;
   for(it = items.begin(); it != items.end(); it++) {
      dump.push_back(it->first + "=" + (it->second ? "true" : "false"));
   }
}

std::string CFormValues::set::get(const std::string& xpath)
{
   if (xpath.size() == 0) {
      std::ostringstream ss;
      typeof(items.begin()) it;
      for (it = items.begin(); it != items.end(); it++) {
         if (it->second) ss << '\n' << it->first;
      }
      return ss.str();
   }
   typeof(items.begin()) it = items.find(xpath);
   if (it == items.end()) return ""; // throw... wrong xpath
   else return it->second ? "1" : "0";
}

void CFormValues::set::get(std::map<std::string, std::string>& fieldmap)
{
   std::ostringstream ss;
   bool first = true;
   typeof(items.begin()) it;
   for(it = items.begin(); it != items.end(); it++) {
      if (it->second) {
         //if (first) ss << it->first;
         //else {
            ss << '\n' << it->first;
            first = false;
         //}
      }
   }
   if (first) ss << "\n";
   fieldmap[name] = ss.str();
}

void CFormValues::set::setValue(const std::string& _value)
{
   std::istringstream ss(_value);
   std::string item;
   while(std::getline(ss, item, '\n')) {
      if (item == "") continue;
      typeof(items.begin()) it = items.find(item);
      if (it != items.end()) it->second = true;
      //std::cout << name << "." << it->first << "=" << it->second << std::endl;
   }
}

void CFormValues::set::setValue(const std::string& xpath, const std::string& _value)
{
   //std::cout << "set::setValue " << xpath << std::endl;
   typeof(items.begin()) it = items.find(xpath);
   if (it == items.end()) return; // throw... wrong xpath
   else {
      std::string tmp;
      tmp.resize(_value.size());
      std::transform(_value.begin(), _value.end(), tmp.begin(), tolower);

      if (tmp.size() == 1 && isdigit(tmp[0]) && tmp[0] != '0') it->second = true;
      else if (tmp=="t" || tmp == "y" || tmp=="true" || tmp=="on" || tmp=="yes") it->second = true;
      else it->second = false;

      //std::cout << "  " << name << "." << it->first << "=" 
      //   << _value << "=" << tmp << "=" << it->second << std::endl;
   }
}

void CFormValues::set::setState(const std::string& option, bool on)
{
   throw Exception("TODO: set::setState not implemented");
}

void CFormValues::set::setState(const std::string& option, const std::string& _value)
{
   throw Exception("TODO: set::setState not implemented");
}

bool CFormValues::set::getState(const std::string& option)
{
   throw Exception("TODO: set::getState not implemented");
   return false;
}

std::string CFormValues::choice::get(const std::string& xpath)
{
   if (xpath.size() == 0) {
      std::ostringstream ss;
      typeof(items.begin()) it;
      for (it = items.begin(); it != items.end(); it++) {
         if (it->second) return it->first;
      }
      return "";
   }
   return CFormValues::set::get(xpath);
}

void CFormValues::choice::setState(const std::string& option, bool on)
{
   throw Exception("TODO: choice::setState not implemented");
}

void CFormValues::choice::setState(const std::string& option, const std::string& _value)
{
   throw Exception("TODO: choice::setState not implemented");
}

bool CFormValues::choice::getState(const std::string& option)
{
   throw Exception("TODO: choice::getState not implemented");
   return false;
}

void CFormValues::clear()
{
   typeof(fields.begin()) it;
   for(it = fields.begin(); it != fields.end(); it++) {
      it->second->clear();
   }
}

void CFormValues::apply(const std::map<std::string, std::string>& newfields)
{
   clear();
   typeof(newfields.begin()) itnew;
   for(itnew = newfields.begin(); itnew != newfields.end(); itnew++) {
      //std::cout << " apply " << itnew->first;
      typeof(fields.begin()) it = fields.find(itnew->first);
      if (it != fields.end()) {
         //std::cout << " ok ";
         it->second->setValue(itnew->second);
      }
      //std::cout << std::endl;
   }
}

void CFormValues::setValue(const std::string& xpath, const std::string& value)
{
   std::string main, sub;
   size_t pos = xpath.find('/');
   if (pos == std::string::npos) main = xpath;
   else {
      main = xpath.substr(0, pos);
      sub = xpath.substr(pos+1);
      //std::cout << "xpath: " << xpath << " ... " << main << " & " << sub << std::endl;
   }
   typeof(fields.begin()) it = fields.find(main);
   if (it != fields.end()) {
      if (sub.size() == 0) it->second->setValue(value);
      else it->second->setValue(sub, value);
   } // else throw ...
}

void CFormValues::clearValue(const std::string& name)
{
   typeof(fields.begin()) it = fields.find(name);
   if (it != fields.end()) {
      it->second->clear();
   } // else throw ...
}

std::string CFormValues::get(const std::string& xpath)
{
   std::string main, sub;
   size_t pos = xpath.find('/');
   if (pos == std::string::npos) main = xpath;
   else {
      main = xpath.substr(0, pos);
      sub = xpath.substr(pos+1);
   }
   typeof(fields.begin()) it = fields.find(main);
   if (it != fields.end()) {
      return it->second->get(sub);
   }
   return "";
}

double CFormValues::getFloat(const std::string& xpath)
{
   std::string main, sub;
   size_t pos = xpath.find('/');
   if (pos == std::string::npos) main = xpath;
   else {
      main = xpath.substr(0, pos);
      sub = xpath.substr(pos+1);
   }
   typeof(fields.begin()) it = fields.find(main);
   if (it != fields.end()) {
      return it->second->getFloat(sub);
   }
   return 0;
}

long CFormValues::getInt(const std::string& xpath)
{
   std::string main, sub;
   size_t pos = xpath.find('/');
   if (pos == std::string::npos) main = xpath;
   else {
      main = xpath.substr(0, pos);
      sub = xpath.substr(pos+1);
   }
   typeof(fields.begin()) it = fields.find(main);
   if (it != fields.end()) {
      return it->second->getInt(sub);
   }
   return 0;
}

void CFormValues::get(std::map<std::string, std::string>& fieldmap)
{
   fieldmap.clear();
   typeof(fields.begin()) it;
   for(it = fields.begin(); it != fields.end(); it++) {
      it->second->get(fieldmap);
   }
}

void CFormValues::add(field* pfield)
{
   fields[pfield->name] = pfield;
}

void CFormValues::dump(std::vector<std::string>& dump)
{
   typeof(fields.begin()) it;
   for(it = fields.begin(); it != fields.end(); it++) {
      it->second->dump(dump);
   }
}

}} // namespace
// vim:sw=3:ts=8:et
