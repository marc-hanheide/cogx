/**
 * @file TrackRectangles.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief Tracking rectangles
 **/

#include "GestaltPrinciple.hh"
#include "RectangleDefinition.hh"

namespace Z
{

/**
 * @brief Tracking rectangles
 */
class TrackRectangles : public GestaltPrinciple
{
private:

	/**
	*	@brief Rectangles to track
	*/
	class TRectangle
	{
		private:
	
		public:
//		unsigned object_id;																///< ID of the object
		unsigned id;																			///< Original ID of the L-Junction
		unsigned age;																			///< Age of the stored TLJct (in steps)
		unsigned id_former_tracked;												///< ID of the tracked L-Junction (age+1)
		unsigned id_next_tracked;													///< id of the L-Junction from next image (age-1)
	
		RectDef rectDef;																	///< Properties of the L-Junction
	
		TRectangle(unsigned i, RectDef rd);
		void FormerTracked(unsigned i);										///< Mark former LJct (age+1)
		void NextTracked(unsigned id);										///< Mark next Ljct (age-1)
	};

	bool firstCall;

  Array<TRectangle*> tRects;													///< Candidates (rectangles) for tracking

  void Rank();
	void Mask();

//	unsigned maxAge;																		///< maximum age for tracking
	void AgeTObjects(unsigned maxAge);
	void AgeTRectangles(unsigned maxAge);

	void SaveNewRectangle(unsigned idx);
	void SaveNewRectangleNonIncremental();
	void TrackRectangleNonIncremental();
	void TrackRectangleIncremental(unsigned idx);

public:
  TrackRectangles(Config *cfg);
  virtual void Operate(bool incremental);
	virtual void OperateNonIncremental();
  virtual bool NeedsOperate();
  virtual void Reset(const Image *img);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
