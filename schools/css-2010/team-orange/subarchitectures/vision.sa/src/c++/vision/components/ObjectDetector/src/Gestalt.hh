/**
 * @file Gestalt.hh
 * @author Zillich
 * @date Februar 2007
 * @version 0.1
 * @brief Prototype of Gestalts
 **/

/**
 * TODO: - function AssignIDs(), call whenever an element was deleted or moved
 */

#ifndef Z_GESTALT_HH
#define Z_GESTALT_HH

#include "Namespace.hh"
#include "Array.hh"

namespace Z
{

/**
 *	@brief Class Gestalt is the prototype of Gestalts.
 */
class Gestalt
{
public:
  enum Type
  {
    SEGMENT,
    LINE,
    ARC,
    PARALLEL_LINE_GROUP,
    CONVEX_ARC_GROUP,
    ELLIPSE,
    BALL,
    COLLINEARITY,
    T_JUNCTION,
    L_JUNCTION,
    E_JUNCTION,
    EXTELLIPSE,
    CYLINDER,
    CONE,
    CORNER,
    CLOSURE,
    EXTCLOSURE,
    RECTANGLE,
    EXTRECTANGLE,
    FLAP,
    CUBE,
		WALL_LINE,
    WALL,
		EXIT_,
		MOTION_FIELD_ELEMENT,
		MOTION_FIELD,
		TRKT_RECTANGLE,
		TRKT_FLAP,
		TRKT_CUBE,
    OBJECT,
		BEST_RESULT,
		TRACKEDCUBE,				/// TODO Sollten eigentlich TRACKED_XXX heißen
		TRACKEDCONE,
		TRACKEDCYLINDER,
		TRACKEDBALL,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

protected:
  Type type;
  unsigned id;
  unsigned rank;

public:
  double acc;						///< probability of accidental occurrance
  double sig;						///< significance
  unsigned masked;			///< TODO: might be masked by more gestalts or other constraints
  double weight;				///< weight of what ???
//	bool clear;						///< clear Gestalts bevor processing of the next image
//	unsigned age;					///< Age of the Gestalt (in processed images), for uncleared Gestalts (tracking)

public:
  static const char* TypeName(Type t);
  static Type EnumType(const char *type_name);
	
	/**
	 *	@brief Constructor
	 *	@param t Type of Gestalt
	 */
  Gestalt(Type t);

	/**
	 *	@brief Destructor
	 */
  virtual ~Gestalt() {}

	/**
	 *	@brief Get type of the Gestalt
	 *	@return Returns the type of the Gestalt (Type).
	 */
  Type GetType() {return type;}

	/**
	 *	@brief Get the unique ID of the Gestalt.
	 *	@return Returns the ID of the Gestalt
	 */
  unsigned ID() {return id;}

	/**
	 *	@brief Set the unique ID of the Gestalt.
	 *	@param i New ID of the Gestalt.
	 */
  void SetID(unsigned i) {id = i;}

	/**
	 *	@brief Get the rank of the Gestalt.
	 *	@return Returns the rank of the Gestalt.
	 */
  unsigned Rank() {return rank;}

	/**
	 *	@brief Set Rank of the Gestalt.
	 *	@param r New Rank of the Gestalt.
	 */
  void SetRank(unsigned r) {rank = r;}

	/**
	 *	@brief Mask the Gestalt
	 *	@param by ID of the masking Gestalt.
	 */
  void Mask(unsigned by) {masked = by;}

	/**
	 *	@brief Check, if the Gestalt is masked.
	 *	@return Returns true, if Gestalt is masked.
	 */
  bool IsMasked() {return masked != UNDEF_ID;}

	/**
	 *	@brief Check, if the Gestalt is unmasked.
	 *	@return Returns true, if Gestalt is unmasked.
	 *	TODO Sinnlos, weil es auch IsMasked gibt (??)
	 */
  bool IsUnmasked() {return masked == UNDEF_ID;}

	/**
	 *	@brief Draw the Gestalt in detail
	 *	@param detail Degree of detail for drawing the gestalt (default = 0).
	 */
  virtual void Draw(int detail = 0) {}

	/**
	 *	@brief Draw info for the Gestalt-window
	 */
  virtual void DrawInfo() {}

	/**
	 *	@brief Returns the text
	 */
  virtual const char* GetInfo();

	/**
	 *	@brief Returns true, if the Gestalt is at the position x/y.
	 *	@param x X-coordinate of the Gestalt.
	 *	@param y Y-coordinate of the Gestalt.
	 *	@return Returns true, if the Gestalt is at position x/y (default = false).
	 */
  virtual bool IsAtPosition(int x, int y) {return false;}

//	/**
//	 *	@brief Rises the age of uncleared Gestalts.
//	 */
// 	virtual void AgeGestalt() {}
};

}

#endif
