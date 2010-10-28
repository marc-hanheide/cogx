/**
 * @file FormMotionField.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief The class caculates the motion field
 **/

#include "GestaltPrinciple.hh"
#include "LJunction.hh"
#include "LJunctionDefinition.hh"
#include "MotionDefinition.hh"
#include "MotionFieldElement.hh"

namespace Z
{

/**
 * @brief The class caculates the motion field.
 */
class FormMotionField : public GestaltPrinciple
{
private:

	/**
	*	@brief L-Junction to track
	*/
	class TLJct
	{
		private:
	
		public:
//		unsigned object_id;																///< ID of the object
		unsigned id;																			///< Original ID of the L-Junction
		unsigned age;																			///< Age of the stored TLJct (in steps)
		unsigned id_former_tracked;												///< ID of the tracked L-Junction (age+1)
		unsigned id_next_tracked;													///< id of the L-Junction from next image (age-1)
	
		LJctDef lJctDef;																	///< Properties of the L-Junction
	
		TLJct(unsigned i, LJctDef ld);
		void FormerTracked(unsigned i);										///< Mark former LJct (age+1)
		void NextTracked(unsigned id);										///< Mark next Ljct (age-1)
	};

	/**
	*	@brief Motion Field
	*/
	class TMotionField
	{
		private:
	
		public:
		unsigned id;																			///< Original ID of the L-Junction
		unsigned age;																			///< Age of the stored TLJct (in steps)
	
		MotionDef motDef;																	///< Properties of the L-Junction
	
		TMotionField(unsigned i, MotionDef md);
	};

  Array<TLJct*> tLJcts;																///< L-Junctions for tracking
	Array<TMotionField*> motField;											///< Stored motion fields

//	Array<LJunction*> savLJcts;	/// save L-Junctions

  bool needsOperate;
//	unsigned maxAge;																		///< Maximum age of tracked objects
	bool firstCall;
	Vector2 meanDirection; 															///< Mean direction of the motion field => used for masking bad elements
	Vector2 FOE;																				///< Focus of expansion - point
	double sigFOE;																			///< Significance for FOE
	double expFOE;																			///< Expansion value for FOE

  void Rank();
	void Mask();

	// Age of Objects
	void AgeTObjects(unsigned maxAge);
	void AgeTLJcts(unsigned maxAge);
	void AgeMotionField(unsigned maxAge);

	// Motion field processing
	void SaveLJunction(unsigned idx);
	void TrackLJunction(unsigned idx);
	void CreateNewMotionField();
	void CalculateFocusOfExpansion();
	void CreateGestaltMotionField();


public:
  FormMotionField(Config *cfg);
	virtual void OperateNonIncremental();
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
