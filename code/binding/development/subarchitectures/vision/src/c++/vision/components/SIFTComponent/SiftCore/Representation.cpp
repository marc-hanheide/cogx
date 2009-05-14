#include <iostream>
#include <fstream>
#include "Representation.h"
#include "siftGlobal.h"

using namespace std;

Representation::Representation() {
}

Representation::~Representation(){
  int i;

  for (i=0; i<v_pFeatures.size(); i++)
    delete(v_pFeatures[i]);
  v_pFeatures.clear();
}


//------------------------------
void Representation::push_back(FeatureVector *pfeatures){
  v_pFeatures.push_back(pfeatures);
}

//-----------------------------
int Representation::save(ofstream &file) {

  int inum;
  float fnum;

  int nVectors = v_pFeatures.size();
  file.write((char*)&nVectors,sizeof(nVectors));
  
  for (int i=0; i<nVectors; i++) {

    // write i-th feature to the stream
    FeatureVector *pFeatureVector = v_pFeatures[i];
    
    int nFeatures = (*pFeatureVector).size();
    file.write((char*)&nFeatures,sizeof(nFeatures));

    for (int f=0; f<nFeatures; f++) {

      inum = (*pFeatureVector)[f].loc.x;
      file.write((char*)&inum,sizeof(inum));
      inum = (*pFeatureVector)[f].loc.y;
      file.write((char*)&inum,sizeof(inum));
      fnum = (*pFeatureVector)[f].fScale;
      file.write((char*)&fnum,sizeof(fnum));
      fnum = (*pFeatureVector)[f].fOrientation;
      file.write((char*)&fnum,sizeof(fnum));
      

      for (int j = 0;  j < SIFT_LENGTH;  ++j) {
	unsigned char c=(*pFeatureVector)[f].descriptor[j];
	file.write((char*)&c,sizeof(c));
      }

      inum = (*pFeatureVector)[f].dxCenter;
      file.write((char*)&inum,sizeof(inum));
      inum = (*pFeatureVector)[f].dyCenter;
      file.write((char*)&inum,sizeof(inum));
    }

  }

  return RETURN_OK;
}

//-----------------------------
int Representation::load(ifstream &file) {

  int nVecs;
  file.read((char *)&nVecs, sizeof(nVecs));

  //cout << "nfeatureVectors "  << nVecs << endl;

  FeatureVector *pfeatureVector = NULL;
  for (int iVec = 0;  iVec < nVecs;  iVec++) {

    pfeatureVector = new FeatureVector();

    int nFeatures;
    file.read((char *)&nFeatures, sizeof(nFeatures));

    for (unsigned int i = 0;  i < nFeatures;  ++i) {
      Feature feat;
      int x, y;
      file.read((char *)&x, sizeof(x));
      file.read((char *)&y, sizeof(y));
      
      feat.loc.x = x;
      feat.loc.y = y;
      
      float fScale, fOrientation;
      file.read((char *)&fScale, sizeof(fScale));
      file.read((char *)&fOrientation, sizeof(fOrientation));

      feat.fScale = fScale;
      feat.fOrientation = fOrientation;
      
      for (int j = 0;  j < SIFT_LENGTH;  ++j) {
	file.read((char *)&(feat.descriptor[j]), sizeof(char));
      }
      
      // NEW 
      int dx, dy;
      
      file.read((char *)&dx, sizeof(dx));
      file.read((char *)&dy, sizeof(dy));
      
      feat.dxCenter = dx;
      feat.dyCenter = dy;
      
      (*pfeatureVector).push_back(feat);
    }
    
    push_back(pfeatureVector);
  }  // for (iVec)
  
  return RETURN_OK;
}



//--------------------------------
bool Representation::has_singleFeatureVector(FeatureVector *&pfeatureVector) {
  if (v_pFeatures.size()==1) {
    pfeatureVector = v_pFeatures[0];
    return true;
  }
  pfeatureVector = NULL;
  return false;
}


//----------------------------
int Representation::get_pfeatureVector(int iVec, 
				       FeatureVector *&pfeatureVector) {
  
  if ((iVec<0) || (iVec>=v_pFeatures.size()))
    return ERROR_VECTOR_OUTOFBOUND;
  else {
    pfeatureVector = v_pFeatures[iVec];
    return RETURN_OK;
  }
}
