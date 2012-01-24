/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */


#include "RASDescriptor.hh"


namespace P 
{


/********************** RASDescriptor ************************
 * Constructor/Destructor
 */
RASDescriptor::RASDescriptor()
  : data(0), size(0)
{
}

RASDescriptor::~RASDescriptor()
{
  if (data!=0) delete[] data;
}



/**
 * compare two ras-descriptors
 */
float RASDescriptor::Compare(RASDescriptor *ras1, RASDescriptor *ras2)
{
  if (ras1->Size() != ras2->Size())
    return FLT_MAX;

  float dif;
  float distsq = 0;

  for (unsigned i = 0; i < ras1->Size(); i++) 
  {
    dif = ras1->data[i] - ras2->data[i];
    distsq += dif * dif;
  }

  return distsq;
}


/**
* Save a ras descriptor
*/
void RASDescriptor::SaveDescriptor(ofstream &os, RASDescriptor *ras, char *name)
{
  if (name!=0)
    os<<name<<" ";
  else
    os<<"noname ";

  os<<ras->sa<<" "<<ras->ss<<" ";

  for (unsigned i=0; i<ras->Size(); i++)
    os<<ras->data[i]<<" ";
  os<<'\n';
}

/**
* Load a ras descriptor
*/
void RASDescriptor::LoadDescriptor(ifstream &is, RASDescriptor *ras, string &name)
{
  is>>name;

  int sa, ss;

  is>>sa>>ss;

  ras->Set(sa,ss);
  
  for (unsigned i=0; i<ras->Size(); i++)
    is>>ras->data[i];
}


}

