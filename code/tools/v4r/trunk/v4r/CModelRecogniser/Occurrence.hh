/**
 * $Id$
 */

#ifndef P_OCCURRENCE_HH
#define P_OCCURRENCE_HH



namespace P
{

class Occurrence
{
public:
  unsigned object, view, key;
  Occurrence() {};
  Occurrence(unsigned oidx, unsigned vidx, unsigned kidx) : object(oidx), view(vidx), key(kidx) {}
};





} //--END--

#endif

