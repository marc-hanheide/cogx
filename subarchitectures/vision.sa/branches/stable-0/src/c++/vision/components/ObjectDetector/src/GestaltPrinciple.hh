/**
 * @file GestaltPrinciple.hh
 * @author Michael Zillich
 * @date November 2006
 * @version 0.1
 * @brief Header Class of the prototype Gestalt-Principle
 **/

#ifndef Z_GESTALT_PRINCIPLE_HH
#define Z_GESTALT_PRINCIPLE_HH

#include "Config.hh"
#include "Image.hh"
#include "Gestalt.hh"

namespace Z
{

/**
 *	@brief Class GestaltPrinciple
 */
class GestaltPrinciple
{
public:
  enum Type
  {
    FORM_SEGMENTS,
    FORM_LINES,
    FORM_ARCS,
    FORM_PARALLEL_LINE_GROUPS,
    FORM_CONVEX_ARC_GROUPS,
    FORM_ELLIPSES,
		FORM_BALLS,
    FORM_JUNCTIONS,
    FORM_EXTELLIPSES,
    FORM_CYLINDERS,
    FORM_CONES,
    FORM_CORNERS,
    FORM_CLOSURES,
    FORM_EXTCLOSURES,
    FORM_RECTANGLES,
    FORM_EXTRECTANGLES,
    FORM_FLAPS,
    FORM_CUBES,
		FORM_WALLLINES,
    FORM_WALLS,
		FORM_EXITS,
		FORM_MOTION_FIELD,
		TRACK_RECTANGLES,
		TRACK_FLAPS,
		TRACK_CUBES,
    TRACK_OBJECTS,
		FORM_BEST_RESULTS,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

private:
  double runtime;   								///< runtime in [s]

  struct timespec startTime;				///< start time [timespec]
  struct timespec endTime;					///< end time [timespec]

protected:
	unsigned maxAge;									///< maximum age for tracking, motion field ...

  Config *config;										///< configuration of vs3
  Array<unsigned> next_principles;  ///< next principles, to be informed of new gestals

public:
	/**
	 *	@brief Rank all Gestalts by significance value with the compare-function
	 *	@TODO ARI: This function was protected
	 */
  void RankGestalts(Gestalt::Type type, int(*compar)(const void *, const void *));

	/**
	 *	@brief TypeName
	 */
	static const char* TypeName(Type t);

	/**
	 *	@brief EnumType
	 */
  static Type EnumType(const char *type_name);

	/**
	 *	@brief Constructor
	 */
  GestaltPrinciple(Config *cfg);

	/**
	 *	@brief Destructor
	 */
  virtual ~GestaltPrinciple() {}

	/**
	 *	@brief Resets the Gestalt-Principle
	 */
  virtual void Reset(const Image *img) {};

	/**
	 *	@brief Operate a Gestalt-Principle in incremental or non-incremental way. This method is called directly from vision core,
	 *				after the start of processing in order of the principles.
	 */
  virtual void Operate(bool incremental) {};

	/**
	 *	@brief Operates a Gestalt in a non-incremental way. The method is called directly from vision core, after the
	 *				incremental principles are processed and the given processing time is over.
	 */
	virtual void OperateNonIncremental() {}								// used for operating after incremental operating

	/**
	 *	@brief returns true, if the Gestalt-principle needs processing time.
	 */
  virtual bool NeedsOperate() {return true;}

	/**
	 *	@brief Informs the Gestalt-Principle about a new Gestalt, to hypothesise new Gestalts.
	 *	@param type Type of Gestalt
	 *	@param idx Index of Gestalt
	 */
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx) {}

	/**
	 *	@brief Informs a Gestalt-Principle about a new Candidate for extracting 
	 *	@param	l line?
	 *	@param	side side of line?
	 *	TODO Is this function used anymore?
	 */
  virtual void InformNewCandidate(unsigned l, unsigned side){}

	/**
	 *	@brief Called for drawing the Gestalts in different degrees of details.
	 *	@param detail Degree of detail for the drawn Gestalt
	 */
  virtual void Draw(int detail = 0) {}

	/**
	 *	@brief Setting the properties of the canny edge detector. (only used for segementing)
	 *	@param a Alpha value of the canny edge detector
	 *	@param o Omega value of the canny edge detector
	 */
  virtual void SetCanny(int a, int o) {}

	/**
	 *	@brief Returns the processing time of the Gestalt-Principle.
	 *	@return Total runtime of the processing time for the Gestalt-Principle.
	 */
  double RunTime() {return runtime;}

	/**
	 *	@brief Reset the runtime of the Gestalt-Principle.
	 */
  void ResetRunTime() {runtime = 0.;}

	/**
	 *	@brief Set the runtime of the Gestalt-Principle.
	 *	@param t runtime to set
	 *	TODO used?
	 */
  void SetRunTime(double t) {runtime = t;}

	/**
	 *	@brief Add runtime to the Gestalt-Principle.
	 *	@param t runtime to add
	 *	TODO used?
	 */
  void AddRunTime(double t) {runtime += t;}

  void StartRunTime();

	void StopRunTime();
};

}

#endif
