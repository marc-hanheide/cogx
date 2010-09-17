/**
 * $Id: FormConvexArcGroups.hh,v 1.20 2007/02/04 23:53:03 mxz Exp mxz $
 * TODO:
 * - create tangents with the expected number of intersections, so that no
 *   later resizing of the arrays is necessary.
 * - avoid generation of double groups, using signature
 */

#ifndef Z_FORM_CONVEX_ARC_GROUPS_HH
#define Z_FORM_CONVEX_ARC_GROUPS_HH

#include "Array.hh"
#include "VoteImage.hh"
#include "Gestalt.hh"
#include "ConvexArcGroup.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

class ArcLink
{
public:
  unsigned id;  ///< ID of the other intersecting tangent
  double sig;
  ArcLink() {}
  ArcLink(unsigned i, double s) : id(i), sig(s) {}
};

class FormConvexArcGroups : public GestaltPrinciple
{
private:
  int SEARCH_METHOD;
  int GROUP_CRITERION;
  static const int HASH_SIZE = 29989;  // TODO: this is quite arbitrary
  unsigned hashtable[HASH_SIZE];
  Array< Array<ArcLink> > links;
  Array<ConvexArcGroup*> cands;
  unsigned cands_created;
  bool done;

  double Evaluate(Array<unsigned> &arcs, unsigned l, unsigned u, int criterion);
  void NextEnd(Array<unsigned> &arcs, unsigned l, unsigned u,
      unsigned &end, unsigned &next, int criterion);
  double GrowBest(Array<unsigned> &arcs, unsigned &l, unsigned &u);
  void GrowAll(Array<unsigned> &arcs, unsigned l, unsigned u,
      Array<unsigned> &arcs_opt, unsigned &size_opt, double q_opt);
  void DrawSearchLines(unsigned arc);
  void CheckSearchLines(unsigned arc);
  bool CreateIntersection(unsigned i, unsigned j);
  void CreateArcJunctions(unsigned arc, unsigned end,
    Array<VoteImage::Elem> &iscts);
  void Prepare();
  void Create();
  void Rank();
  void ClearHashTable();
  unsigned Hash(Array<unsigned> &arcs);
  unsigned Hash(Array<unsigned> &arcs, unsigned l, unsigned u);
  void ExhaustiveSearch(unsigned arc, Array<unsigned> &arcs,
      Array<unsigned> &arcs_opt);
  void GreedySearch(unsigned arc, Array<unsigned> &arcs);
  void NewCand(ConvexArcGroup *g);
  void InstantiateBestCand();

public:
  static VoteImage *vote_img;

  FormConvexArcGroups(Config *cfg);
  virtual ~FormConvexArcGroups();
  virtual void Reset(const Image *img);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate() {return !done;}
  virtual void Draw(int detail = 0);
};
 
}

#endif

