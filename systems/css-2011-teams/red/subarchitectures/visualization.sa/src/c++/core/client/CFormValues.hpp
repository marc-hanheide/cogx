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
#include <vector>
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
      field(const std::string& fieldname="");
      virtual void clear();
      virtual void setValue(const std::string& _value);
      virtual void setValue(const std::string& xpath, const std::string& _value);
      virtual std::string get(const std::string& xpath);
      virtual double getFloat(const std::string& xpath);
      virtual long   getInt(const std::string& xpath);
      virtual void get(std::map<std::string, std::string>& fieldmap);
      virtual void dump(std::vector<std::string>& dump);

      virtual void setState(const std::string& option, bool on); // throws
      virtual void setState(const std::string& option, const std::string& _value); // throws
      virtual bool getState(const std::string& option); // throws
   };

   class valuelist {
   public:
      std::vector<std::string> items;
      valuelist() {}
      valuelist(const std::vector<std::string>& values) {
         items = values;
      }
      valuelist& operator << (const std::string& value) {
         items.push_back(value);
         return *this;
      }
   };

   class set: public field
   {
   public:
      std::map<std::string, bool> items;
      set(const std::string& name, const valuelist& setitems);
      void clear();
      virtual void dump(std::vector<std::string>& dump);
      virtual std::string get(const std::string& xpath);
      virtual void get(std::map<std::string, std::string>& fieldmap);
      virtual void setValue(const std::string& _value);
      virtual void setValue(const std::string& xpath, const std::string& _value);
      virtual void setState(const std::string& option, bool on);
      virtual void setState(const std::string& option, const std::string& _value);
      virtual bool getState(const std::string& option);
   };

   // A set in which at most one element can be chosen
   class choice: public set
   {
   public:
      choice(const std::string& name, const valuelist& setitems): set(name, setitems) {}
      virtual std::string get(const std::string& xpath);
      virtual void setState(const std::string& option, bool on);
      virtual void setState(const std::string& option, const std::string& _value);
      virtual bool getState(const std::string& option);
   };

   // TODO: add destructor!!
   std::map<std::string, field*> fields;
   void clear();
   void apply(const std::map<std::string, std::string>& newfields);
   void setValue(const std::string& xpath, const std::string& value);
   void clearValue(const std::string& name);
   // TODO: change the CFormValues interface! Drop xpath
   //   - get(fieldname), getFloat(), getInt()
   //   - getState(fieldname, option) 
   //   - getStateMask(fieldname) [up to 64 options], getList()
   //   - set(fieldname)
   //   - setState(fieldname, option, bool/string)
   std::string get(const std::string& xpath);
   double getFloat(const std::string& xpath);
   long   getInt(const std::string& xpath);
   void get(std::map<std::string, std::string>& fieldmap);
   void add(field* pfield);
   void dump(std::vector<std::string>& dump);
};

}} // namespace
#endif /* end of include guard: CFORMVALUES_QQ2XCJUG */
// vim:sw=3:ts=8:et
