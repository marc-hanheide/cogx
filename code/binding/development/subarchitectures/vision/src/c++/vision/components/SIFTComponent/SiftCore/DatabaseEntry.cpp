#include <iostream>
#include <fstream>
#include "DatabaseEntry.h"
#include "ioutil.h"
#include "siftGlobal.h"

using namespace std;


DatabaseEntry::DatabaseEntry() {

}

DatabaseEntry::DatabaseEntry(string _strObject) {
  strObject = _strObject;
}


DatabaseEntry::~DatabaseEntry() {

  map<string, Representation *>::iterator iter;  
  for (iter=map_viewreps.begin(); iter!=map_viewreps.end(); ++iter) {
    Representation *prep = iter->second;
    delete(prep);
  }
  map_viewreps.clear();
}



int DatabaseEntry::add_viewrep(string viewName, Representation * prep) {
  
  map_viewreps[viewName] = prep;
  return RETURN_OK;
}


// should make sure viewname exist
void DatabaseEntry::add_features(string viewName, FeatureVector *pfeatures) {
  
  map<string, Representation *>::iterator iter =  map_viewreps.find(viewName);
  
  if (iter ==  map_viewreps.end()) { // this view does not exist
    
    Representation *prep = new Representation();
    prep->push_back(pfeatures);
    add_viewrep(viewName, prep);    

    cout << "adding new view " << viewName.c_str() << endl;
  }
  else { 
    //Representation *prep = map_viewreps[viewName];
    Representation *prep = (*iter).second;
    prep->push_back(pfeatures);

    cout << "adding features to view " << viewName.c_str() << endl;
  }
}

//-----------------------------
int DatabaseEntry::save(ofstream &file) {

  //file << strObject << " "; // obejctname
  writeString(file, strObject);

  int nViews = map_viewreps.size();
  //file << nViews << " "; // number of views
  file.write((char*)&nViews, sizeof(nViews));


  map<string, Representation *>::iterator iter;
  for (iter=map_viewreps.begin(); iter!=map_viewreps.end(); iter++) {

    //file << (*iter).first << " "; // viewname    
    writeString(file, (*iter).first);
    
    Representation *prep= (*iter).second;
    prep->save(file);
  }

  return RETURN_OK;
}



//--------------------------------
int DatabaseEntry::load(ifstream &file) {
  
  string objlabel;    
  readString(file, objlabel); 

  //cout << "objlabel " << objlabel << ", ";  
  strObject = objlabel;
    
  int nViews;    
  file.read((char*)&nViews, sizeof(nViews));
    
  //cout << "objViews " << nViews << ", ";

  for (int iView=0; iView<nViews; iView++) {
      
    string viewname;
    readString(file, viewname);
    
    //cout << "viewname " << viewname << ", ";
    
    Representation *prep = new Representation();
    prep->load(file);
    
    // add view-representation
    add_viewrep(viewname, prep);
  }
  
  return RETURN_OK;
}




//----------------------------
bool DatabaseEntry::has_singleView(Representation *&prep) {
  map<string, Representation *>::iterator iter;

  if (map_viewreps.size()==1) {
    iter=map_viewreps.begin();
    prep = (*iter).second;
    return true;
  }
  prep = NULL;
  return false;
}
