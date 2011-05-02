/** @file Retina.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_RETINA_H_
#define _GOLEM_DEMO_COMMON_RETINA_H_

//------------------------------------------------------------------------------

#include <Golem/Phys/Renderer.h>
#include <Golem/Demo/Common/Image.h>
#include <Golem/Demo/Common/RigidBodyTracker.h>
#include <Golem/Demo/Common/Embodiment.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Retina input data */
struct RetinaInp {
	/** Objects to be mapped onto retina */
	RigidBodyData::Seq objects;
};

/** Retina output data */
struct RetinaOut {
	/** Retina image (after object alpha blending) */
	Image3 retina;
	/** Objects images (prior to blending) */
	Image3::Seq objects;
};

//------------------------------------------------------------------------------

class Retina;

/** Renders retina */
class RetinaRenderer : public DebugRenderer {
public:
	typedef shared_ptr<RetinaRenderer> Ptr;

	/** Element occupancy array renderer description */
	class Desc {
	public:
		/** Pointer to Retina */
		const Retina *pRetina;
		
		/** Retina bounds colour */
		RGBA boundsColour;
		/** Display retina bounds */
		bool boundsShow;
		/** Retina colour */
		RGBA retinaColour;
		/** Display retina */
		bool retinaShow;
		/** Colour/monochrome */
		bool colourShow;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			boundsColour = RGBA::GREEN;
			boundsShow = true;
			retinaColour = RGBA::WHITE;
			retinaShow = true;
			colourShow = true;
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			//if (pRetina == NULL)
			//	return false;
			
			return true;
		}
	};

	bool create(const Desc &desc);
};

//------------------------------------------------------------------------------

/** Retina
*/
class Retina : public Filter<RetinaInp, RetinaOut>, public Runnable {
public:
	typedef shared_ptr<Retina> Ptr;
	typedef Filter<RetinaInp, RetinaOut> Base;

	/** Receptive field */
	struct ReceptiveField {
		typedef std::vector<ReceptiveField> Seq;
		
		/** 3D position */
		Vec3 position;
		/** Colour */
		RGBA colour;
	};

	/** Object description */
	class Desc : public Base::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Retina, Channel::Ptr, Embodiment&)

	public:
		/** Retina centre pose */
		Mat34 retinaPose;
		/** Number of receptive fields in each dimension */
		Image3::Idx3 receptiveFieldNum;
		/** Receptive field grid */
		Vec3 receptiveFieldGrid;
		/** (Relative) receptive field diameter */
		Real receptiveFieldDiamFac;
		/** Minimal parallels' cluster diameter */
		U32 clusterDiam;
		/** Parallels' job event timeout */
		MSecTmU32 jobTimeOut;

		/** Hierarchical clustering mode */
		bool clustering;
		/** Model objects volume, not only surfaces */
		bool volume;

		/** Retina renderer description */
		RetinaRenderer::Desc retinaRendererDesc;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets receptive field grid from dimensions and number of receptive fields */
		bool setReceptiveFieldGrid(const Vec3 &retinaDimensions, const Image3::Idx3& receptiveFieldNum) {
			if (!retinaDimensions.isPositive() || !receptiveFieldNum.isPositive())
				return false;

			this->receptiveFieldNum = receptiveFieldNum;
			this->receptiveFieldGrid.v1 = retinaDimensions.v1/receptiveFieldNum.n1;
			this->receptiveFieldGrid.v2 = retinaDimensions.v2/receptiveFieldNum.n2;
			this->receptiveFieldGrid.v3 = retinaDimensions.v3/receptiveFieldNum.n3;
			
			return true;
		}
		
		/** Sets number of receptive fields from dimensions and receptive field grid */
		bool setReceptiveFieldNum(const Vec3 &retinaDimensions, const Vec3 &receptiveFieldGrid) {
			if (!retinaDimensions.isPositive() || !receptiveFieldGrid.isPositive())
				return false;
			
			this->receptiveFieldGrid = receptiveFieldGrid;
			this->receptiveFieldNum.n1 = (Image3::Index)Math::ceil(retinaDimensions.v1/receptiveFieldGrid.v1);
			this->receptiveFieldNum.n2 = (Image3::Index)Math::ceil(retinaDimensions.v2/receptiveFieldGrid.v2);
			this->receptiveFieldNum.n3 = (Image3::Index)Math::ceil(retinaDimensions.v3/receptiveFieldGrid.v3);
			
			return true;
		}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Base::Desc::setToDefault();

			name = "Retina";
			
			retinaPose.setId();
			receptiveFieldNum.n1 = 100;
			receptiveFieldNum.n2 = 100;
			receptiveFieldNum.n3 = 100;
			receptiveFieldGrid.set(Real(0.0025), Real(0.0025), Real(0.0025));
			receptiveFieldDiamFac = Real(1.0);
			clusterDiam = 3;
			jobTimeOut = 5000; // 5 sec

			clustering = true;
			volume = false;

			retinaRendererDesc.setToDefault();
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Base::Desc::isValid())
				return false;

			if (!retinaPose.isFinite())
				return false;
			if (!receptiveFieldNum.isPositive() || !receptiveFieldGrid.isPositive() || receptiveFieldDiamFac <= REAL_ZERO)
				return false;
			if (!retinaRendererDesc.isValid())
				return false;
			if (clustering && clusterDiam < 2)
				return false;

			return true;
		}
	};

protected:
	struct Cluster {
		typedef std::vector<Cluster> Seq;
		Image3::Idx3 begin, end;

		Cluster() {
		}
		Cluster(const Image3::Idx3& begin, const Image3::Idx3& end) : begin(begin), end(end) {
		}
	};

	/** Retina bounding box */
	Bounds::Ptr retinaBoundingBox;
	/** Retina pose */
	Mat34 retinaPose;
	/** Retina size */
	Image3::Idx3 receptiveFieldNum;
	/** Receptive field grid */
	Vec3 receptiveFieldGrid;
	Real receptiveFieldGridDiam, receptiveFieldGridDiamInv;
	/** (Relative) receptive field diameter */
	Real receptiveFieldDiam, receptiveFieldDiamInv;
	/** Minimal parallels' cluster diameter */
	U32 clusterDiam;
	/** Parallels' job event timeout */
	MSecTmU32 jobTimeOut;

	/** Hierarchical clustering mode */
	bool clustering;
	/** Model objects volume, not only surfaces */
	bool volume;

	/** Retina renderer description */
	RetinaRenderer::Desc retinaRendererDesc;
	RetinaRenderer::Ptr pRetinaRenderer;
	
	/** Critical section */
	CriticalSection cs;
	/** Active receptive fields */
	ReceptiveField::Seq activeReceptiveFields;

	/** Parallels' parameters */
	const RetinaInp *parallelsInp;
	RetinaOut *parallelsOut;
	/** Parallels' critical sections */
	CriticalSection parallelsCs, parallelsRFCs;
	/** Parallels' threads count */
	U32 parallelsNumOfThreads;
	/** Parallels' semaphore */
	shared_ptr<Semaphore> parallelsSm;
	/** Parallels' jobss count */
	volatile U32 parallelsNumOfJobs;
	/** Parallels' clusters */
	Cluster::Seq parallelsClusters;
	/** Parallels' objects gray levels */
	std::vector<Image3::Element> parallelsGrayLevels;
	/** Parallels' objects gray colours */
	std::vector<golem::RGBA> parallelsGrayColours;
	/** Parallels' index */
	volatile size_t parallelsIndex;
	/** Parallels' step size */
	Real parallelsStep;
	/** Parallels' objects alpha normalisation */
	Real parallelsAlphaNorm;
	
	/** Grayscale colour transformation */
	inline Real RGBToGray(Real r, Real g, Real b) const {
		// grayscale value = 0.299R + 0.587G + 0.114B
		return Real(0.299)*r + Real(0.587)*g + Real(0.114)*b;
	}
	
	/** Hierarchical clustering method. */
	void hierarchicalClustering();
	/** Linear occupancy method. */
	void linearOccupancy();
	
	/** Parallels' job. */
	virtual void run();
	
	/** Renders the object. */
	virtual void render();
	
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** Creates object from description. */
	bool create(const Retina::Desc& desc);
	
	/** Constructor */
	Retina(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~Retina();

	/** Process data */
	virtual bool process(RetinaOut &out, const RetinaInp &inp, MSecTmU32 timeOut = MSEC_TM_U32_INF);

	/** Retina bounding box */
	const BoundingBox* getRetinaBoundingBox() const {
		return static_cast<const BoundingBox*>(retinaBoundingBox.get());
	}

	/** Retina pose */
	const Mat34 &getRetinaPose() const {
		return retinaPose;
	}

	/** Number of receptive fields in each dimension */
	const Image3::Idx3 &getReceptiveFieldNum() const {
		return receptiveFieldNum;
	}

	/** Receptive field grid */
	const Vec3 &getReceptiveFieldGrid() const {
		return receptiveFieldGrid;
	}

	/** (Relative) receptive field diameter */
	Real getReceptiveFieldDiam() const {
		return receptiveFieldDiam;
	}

	/** Active receptive fields */
	const ReceptiveField::Seq& getActiveReceptiveFields() const {
		return activeReceptiveFields;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_RETINA_H_*/
