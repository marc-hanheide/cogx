#ifndef _DATABASE_ENTRY_H_
#define _DATABASE_ENTRY_H_

#include "Recognizer.h"
#include "Representation.h"

class DatabaseEntry {

 public:
  DatabaseEntry();
  DatabaseEntry(string _strObject);
  ~DatabaseEntry();

  /** return 0 is OK, else an error code */
  int add_viewrep(string viewName, Representation * prep);
  void add_features(string viewName, FeatureVector *pfeatures);
  int save(ofstream &file);
  int load(ifstream &file);

  int get_numberViews() {return map_viewreps.size();};

  bool has_singleView(Representation *&prep);
  
  

 public:
  string    strObject; 
  map<string, Representation *> map_viewreps; 
};


#endif 
