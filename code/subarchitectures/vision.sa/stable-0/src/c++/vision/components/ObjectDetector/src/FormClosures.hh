/**
 * @file FormClosures.hh
 * @author Zillich
 * @date March 2007
 * @version 0.1
 * @brief Form convex polygons from lines and junctions.
 **/

#ifndef Z_FORM_CLOSURES_HH
#define Z_FORM_CLOSURES_HH

#include <list>
#include <ext/hash_map>
#include "Array.hh"
#include "GestaltPrinciple.hh"

using namespace __gnu_cxx;

namespace Z
{

/**
 * @brief Gestalt principle class FormClosures.
 */
class FormClosures : public GestaltPrinciple
{
private:
  class ClosCand
  {
  public:
    enum Bending {BEND_LEFT = 0, BEND_RIGHT, BEND_NONE};
    list<unsigned> lines;
    unsigned open[2];
    Bending bend;

    ClosCand(unsigned line0, unsigned line1, unsigned open0, unsigned open1,
        Bending b);
    ClosCand(ClosCand *old);
    unsigned FirstLine();
    unsigned LastLine();
    unsigned FirstOpen();
    unsigned LastOpen();
    bool Closes(unsigned line);
    bool Intersects(unsigned line);
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
  vector<unsigned> prev;    // previous vertex on path
  vector<unsigned> ends;    // ends of lines open for extension
  // jct i is the jct between line i and prev[i]
  vector<unsigned> jcts;    // L-jcts between lines of path
  vector<unsigned> colls;   // collinearities between lines of path
  vector<int> bend;
  vector<unsigned> Q;       // priority queue of all unvisited vertices

  void Rank();
  void HaveNewLJunction(unsigned ljct);
  void HaveNewCollinearity(unsigned coll);
  void HaveNewTJunction(unsigned tjct);
  void UpdateLNeighbors(unsigned ljct);
  void UpdateCNeighbors(unsigned coll);
  void UpdateTNeighbors(unsigned tjct);
  bool IsIsolatedL(unsigned ljct);
  bool IsIsolatedC(unsigned coll);
  bool InSameClosure(unsigned a, unsigned b);
  void NewCandidateL(unsigned jct);
  void NewCandidateC(unsigned coll);
  void ExtendClosuresL(unsigned jct);
  void ExtendClosuresC(unsigned coll);
  void JoinClosuresL(unsigned jct);
  void NewClosure(ClosCand *cand);
  void NewClosure(unsigned first, unsigned last);
  void CreateNAngles(unsigned clos);
  void ShortestPath(unsigned ljct, unsigned coll, unsigned s, unsigned t,
      int end_s, int end_t);
  void ClearPaths(unsigned n);
  void ClearNode(unsigned i);
  int ExtendLJunctions(unsigned u);
  int ExtendLJunctions(unsigned u, int side);
  int ExtendNonConvexLJunctions(unsigned u);
  int ExtendCollinearities(unsigned u);
  bool LineIntersectingPath(unsigned l_new, unsigned j_new, unsigned c_new);
  bool ClosingLineIntersectingPath(unsigned l_close, unsigned l_open,
      unsigned j_init, unsigned c_init);

public:
  FormClosures(Config *cfg);
  virtual void Reset(const Image *img);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}

#endif
