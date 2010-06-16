/**
 * @file FormEJunctions.hh
 * @author Richtsfeld Andreas
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt principle FormEJunctions (ellipse junction).
 **/

#ifndef Z_FORM_E_JUNCTIONS_HH
#define Z_FORM_E_JUNCTIONS_HH

#include "VoteImage.hh"
#include "Line.hh"
#include "Ellipse.hh"
#include "EJunction.hh"
#include "GestaltPrinciple.hh"

namespace Z
{


/**
 * @brief Class FormEJunctions
 */
class FormEJunctions : public GestaltPrinciple
{
private:
  Array<VoteImage::Elem> iscts;
  bool isct_ok[8][8];
  bool first_op;

  void SetupAdmissibilityMatrix();
  bool SetNumLines();
  bool IsctTypeAdmissible(int type_i, int type_j) {return isct_ok[type_i][type_j];}
  void CreateJunctions(unsigned sline, Array<VoteImage::Elem> &iscts);
	
	void PrintCreateJunctions(unsigned sline, Array<VoteImage::Elem> &iscts);		/// TODO only for debugging
	
  void OperateIncremental();
  void OperateNonIncremental();
  void InitLineSearchLines(Line *line);
  void InitEllipseSearchLines(Ellipse *ell);
	void ExtendSmartLines(Gestalt::Type type, unsigned idx);
  void ExtendSearchLines(Line *line);
  void ExtendSearchLines(Ellipse *ell);
	bool NoEJunctionYet(Line *line, Ellipse *ellipse);
	void Mask();

public:
	unsigned baseIndex;			///< Number of different search lines (from vote image)
	unsigned baseOffset;		///< Offset for ellipse search lines (= number of unsplitted lines)
  VoteImage *vote_img;		///< Vote image for forming junctions between ellipses and lines

  FormEJunctions(VisionCore *vc);
  virtual ~FormEJunctions();
	virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
  virtual void Reset();
  virtual void Operate(bool incremental);
  virtual void PostOperate();
  void UpdateEJunctions(unsigned idx);
};

}

#endif


