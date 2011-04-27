/**
 * $Id: FormClosures.hh,v 1.11 2007/03/25 21:35:57 mxz Exp mxz $
 */

#ifndef Z_FORM_CLOSURES_HH
#define Z_FORM_CLOSURES_HH

#include <list>
#include <ext/hash_map>
#include "Array.hh"
#include "GestaltPrinciple.hh"

using namespace __gnu_cxx;

namespace Z
{

class FormClosures : public GestaltPrinciple
{
private:
  class ClosCand
  {
  public:
    enum Bending {BEND_LEFT = 0, BEND_RIGHT, BEND_NONE};
    list<Line*> lines;
    int open[2];
    Bending bend;

    ClosCand(Line *line0, Line *line1, int open0, int open1, Bending b);
    ClosCand(ClosCand *old);
    Line *FirstLine();
    Line *LastLine();
    int FirstOpen();
    int LastOpen();
    bool Intersects(Line *line);
  };

  // using SIG_OFFSET - sig as cost value favours high signature paths as short
  // paths while maintaining positive costs.
  // Note: make sure that offset is larger than the maximum signficance.
  // TODO: maybe there is a nicer way of achieving the same.
  static const double SIG_OFFSET = 100.;
  // not LEFT nor RIGHT
  static const int NONE = RIGHT + 1;

  // map of all closure candidates which a line is the start of
  hash_map< unsigned, list<unsigned> > is_start_of;
  // map of all closure candidates which a line is the end of
  hash_map< unsigned, list<unsigned> > is_end_of;
  Array<ClosCand*> cands;
  vector<double> dist;      // known distances from s to v
  vector<Line*> prev   ;    // previous vertex on path
  vector<int> ends;         // ends of lines open for extension
  // jct i is the jct between line i and prev[i]
  vector<LJunction*> jcts;       // L-jcts between lines of path
  vector<Collinearity*> colls;   // collinearities between lines of path
  vector<int> bend;
  vector<Line*> Q;               // priority queue of all unvisited vertices

  void HaveNewLJunction(unsigned idx);
  void HaveNewCollinearity(unsigned idx);
  void HaveNewTJunction(unsigned idx);
  void UpdateLNeighbors(LJunction *ljct);
  void UpdateCNeighbors(Collinearity *coll);
  void UpdateTNeighbors(TJunction *tjct);
  bool IsIsolatedL(LJunction *ljct);
  bool IsIsolatedC(Collinearity *coll);
  bool InSameClosure(Line *a, Line *b);
  void NewCandidateL(LJunction *jct);
  void NewCandidateC(Collinearity *coll);
  //void ExtendClosuresL(unsigned jct);
  //void ExtendClosuresC(unsigned coll);
  //void JoinClosuresL(unsigned jct);
  //void NewClosure(ClosCand *cand);
  void NewClosure(Line *first, Line *last);
  void CreateNAngles(Closure *clos);
  void ShortestPath(LJunction *ljct, Collinearity *coll, Line *s, Line *t,
      int end_s, int end_t);
  void ClearPaths(unsigned n);
  void ClearNode(unsigned i);
  int ExtendLJunctions(Line *u);
  int ExtendLJunctions(Line *u, int side);
  int ExtendNonConvexLJunctions(Line *u);
  int ExtendCollinearities(Line *u);
  bool LineIntersectingPath(Line *l_new, LJunction *j_new, Collinearity *c_new);
  bool ClosingLineIntersectingPath(Line *l_close, Line *l_open,
      LJunction *j_init, Collinearity *c_init);
  void Mask();

public:
  FormClosures(VisionCore *vc);
  virtual void Reset();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}

#endif

