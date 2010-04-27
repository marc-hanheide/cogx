/**
 * @file TrackCubes.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief Tracking cubes
 **/

#include "GestaltPrinciple.hh"
#include "TCubeDefinition.hh"

namespace Z
{

/**
 * @brief Tracking cubes
 */
class TrackCubes : public GestaltPrinciple
{
private:

	/**
	*	@brief Cubes to track
	*/
	class TCube
	{
		private:
	
		public:
//		unsigned object_id;																///< ID of the object
		unsigned id;																			///< Original ID of the Cube
		unsigned age;																			///< Age of the stored TCube (in steps)
		unsigned id_former_tracked;												///< ID of the tracked TCube (age+1)
		unsigned id_next_tracked;													///< id of the Cube from next image (age-1)
	
		TCubeDef tCubeDef;																///< Properties of the L-Junction
	
		TCube(unsigned i, TCubeDef cd);
		void FormerTracked(unsigned i);										///< Mark former Cube (age+1)
		void NextTracked(unsigned id);										///< Mark next Cube (age-1)
	};

  Array<TCube*> tCubes;																///< Cubes for tracking

	bool firstCall;
	unsigned hypAge;																		///< Maximum age for Gestalt hypothesis
	unsigned nextCubeID;

  void Rank();
	void Mask();

	void AgeTObjects(unsigned maxAge);
	void AgeTCubes(unsigned maxAge);

	void SaveNewCube(unsigned idx);
	void GetCubes();
	void TrackCubeNonIncremental();
	void TrackCubeIncremental(unsigned idx);

// 	void HypothesiseCube(unsigned maxAge);
	void HypothesiseCubeFromFlap(unsigned hypAge);
// 	void HypothesiseCubeFromRectangle(unsigned idx, unsigned maxAge);
	unsigned GetCubeID();


public:
  TrackCubes(Config *cfg);
  virtual void Operate(bool incremental);
	virtual void OperateNonIncremental();
  virtual bool NeedsOperate();
  virtual void Reset(const Image *img);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
