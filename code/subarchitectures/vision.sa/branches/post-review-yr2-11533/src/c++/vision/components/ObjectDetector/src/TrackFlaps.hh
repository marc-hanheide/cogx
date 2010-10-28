/**
 * @file TrackFlaps.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief Tracking flaps.
 **/

#include "GestaltPrinciple.hh"
#include "TFlapDefinition.hh"

namespace Z
{

/**
 * @brief Tracking flaps
 */
class TrackFlaps : public GestaltPrinciple
{
private:

	/**
	*	@brief Flaps to track
	*/
	class TFlap
	{
		private:
	
		public:
//		unsigned object_id;																///< ID of the object
		unsigned id;																			///< Original ID of the Flap
		unsigned age;																			///< Age of the stored Flap (in steps)
		unsigned id_former_tracked;												///< ID of the tracked Flap (age+1)
		unsigned id_next_tracked;													///< id of the Flap from next image (age-1)
	
		TFlapDef flapDef;																	///< Properties of the Flap
	
		TFlap(unsigned i, TFlapDef rd);
		void FormerTracked(unsigned i);										///< Mark former Flap (age+1)
		void NextTracked(unsigned id);										///< Mark next Flap (age-1)
	};

  Array<TFlap*> tFlaps;																///< Flaps for tracking

	bool firstCall;
	unsigned hypAge;
	unsigned nextFlapID;

  void Rank();
	void Mask();

//	unsigned maxAge;																		///< maximum age for tracking
	void AgeTObjects(unsigned maxAge);
	void AgeTFlaps(unsigned maxAge);

	void SaveNewFlap(unsigned idx);
	void GetFlaps();
	void TrackFlapNonIncremental();
	void TrackFlapIncremental(unsigned idx);
	void HypothesiseFlapFromRectangle(unsigned hypAge);
	unsigned GetFlapID();

public:
  TrackFlaps(Config *cfg);
  virtual void Operate(bool incremental);
	virtual void OperateNonIncremental();
  virtual bool NeedsOperate();
  virtual void Reset(const Image *img);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
