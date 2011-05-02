/** @file Layer.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_LAYER_H_
#define _GOLEM_DEMO_COMMON_LAYER_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Collection.h>
#include <Golem/Demo/Common/Rotations.h>
#include <Golem/Demo/Common/ArrN.h>
#include <Golem/Demo/Common/Image.h>
#include <Golem/Demo/Common/ImageFilter.h>
#include <Golem/Demo/Common/Embodiment.h>
#include <deque>
#include <set>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** CRC type used for model and realisation signatures */
typedef CRC32 Signature;

//------------------------------------------------------------------------------

/** 2D pose - angle-position representation */
class Pose2 {
public:
	/** Angle */
	Real a;
	/** Position */
	Vec2 p;

	Pose2() {
	}
	
	Pose2(Real a, const Vec2& p) : a(a), p(p) {
	}

	void set(Real a, const Vec2& p) {
		this->a = a;
		this->p = p;
	}
	
	void setId() {
		a = REAL_ZERO;
		p.setZero();
	}

	void fromMat23(const Mat23& m) {
		m.R.toAngle(a);
		p = m.p;
	}

	void toMat23(Mat23& m) const {
		m.R.fromAngle(a);
		m.p = p;
	}

	bool isFinite() const {
		return Math::isFinite(a) && p.isFinite();
	}
};

//------------------------------------------------------------------------------

class Layer;
class Model;
class Realisation;

/** Part
*	partEnergy = submodelEnergy * 1/(2pi*linVar^2 * sqrt(2pi)*angVar) * exp(- dr^2/linVar^2 - da^2/angVar^2) =
*		= submodelEnergy * C * exp(- Clin * dr^2 - Cang * da^2)
*/
class Part {
public:
	friend class Layer;
	friend class Model;
	friend class Realisation;

	/** Part sequence type */
	typedef std::vector<Part> Seq;
	
	/** Maximum number of different sub-models */
	U32 numOfModels;
	/** Allowable sub-models */
	//std::vector<const Model*> models;
	/** Local pose pointing part centre */
	Pose2 localPose;
	/** Linear pose variance */
	Real linVar;
	/** Angular pose variance */
	Real angVar;
	/** Energy threshold */
	Real threshold;
	/** Image scale units */
	bool imageScale;

protected:
	/** Energy constants */
	Real C, Clin, Cang;

	/** Initialisation */
	void init() {
		C = REAL_ONE/(Math::pow(REAL_2_PI, Real(3.0/2.0))* linVar*linVar * angVar);
		Clin = REAL_ONE/(linVar*linVar);
		Cang = REAL_ONE/(angVar*angVar);
	}

public:
	/** Default constructor sets data to the default values */
	Part() {
		setToDefault();
	}

	/** Sets the parameters to the default values */
	void setToDefault() {
		numOfModels = 1;
		localPose.setId();
		linVar = Real(2.0);
		angVar = Real(0.25)*REAL_PI;
		threshold = Real(0.0);
		imageScale = true;
	}

	/** Checks if the description is valid. */
	bool isValid() const {
		if (numOfModels < 1)
			return false;
		if (!localPose.isFinite())
			return false;
		if (linVar <= REAL_ZERO || angVar <= REAL_ZERO)
			return false;
		if (!Math::isFinite(threshold))
			return false;

		return true;
	}
};

//------------------------------------------------------------------------------

/** Image part model */
class Model {
public:
	friend class Layer;

	/** Model pointer type */
	typedef golem::shared_ptr<Model> Ptr;
	
	/** Object description */
	class Desc {
	public:
		friend class Layer;
		/** Description pointer type */
		typedef golem::shared_ptr<Desc> Ptr;
		/** Description collection */
		typedef std::vector<Ptr> Seq;

	protected:
		/** Creates and initialises Model. */
		virtual Model::Ptr create(Layer& layer) const {
			Model::Ptr pModel(new Model(layer));
			
			if (!pModel->create(*this))
				pModel.release();
			
			return pModel;
		}

	public:
		/** Model signature */
		Signature::Type sig;
		/** Image filter */
		ImageFilter2::Desc::Ptr filterDesc;
		/** Sub-model parts */
		Part::Seq parts;
		/** Energy threshold */
		Real threshold;
		/** Energy normalisation constant */
		Real norm;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		
		virtual ~Desc() {
		}
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			sig = 0;// 0 - auto-generate signature
			//filterDesc.reset(new GaborFilter2::Desc);
			filterDesc.release();
			parts.clear();
			threshold = Real(0.5);
			norm = Real(1.0);
		}

		/** Adds Gabor filter */
		void addGaborFilter(U32 numOfRotations) {
			filterDesc.reset(new GaborFilter2::Desc);
			filterDesc->numOfRotations = numOfRotations;
		}

		/** Adds 2 symmetric parts */
		void add2PartsSymm(Real angle, Real dist) {
			Part part;
			const Vec2 axis(REAL_ZERO, dist);
			const Real angle2 = angle/Real(2.0);
			
			// part #1 (right)
			part.localPose.a = Math::normaliseRad(REAL_PI_2 - angle2);
			Mat22(-angle2).multiply(part.localPose.p, axis);
			parts.push_back(part);
			// part #2 (left)
			part.localPose.a = Math::normaliseRad(-REAL_PI_2 + angle2);
			Mat22(angle2).multiply(part.localPose.p, axis);
			parts.push_back(part);
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (filterDesc != NULL && !parts.empty())
				return false; // no filter and parts simultaneously

			if (filterDesc != NULL) {
				if (!filterDesc->isValid())
					return false;
			}
			else if (!parts.empty()) {
				for (Part::Seq::const_iterator i = parts.begin(); i != parts.end(); i++)
					if (!i->isValid())
						return false;
			}
			else {
				return false; // no filter and no parts
			}

			if (!Math::isFinite(threshold) || norm < REAL_ZERO)
				return false;

			return true;
		}
	};

protected:
	/** Layer in hierarchy */
	Layer& layer;
	/** Model signature */
	Signature::Type sig;
	/** Image filter */
	ImageFilter2::Ptr filter;
	/** Model parts */
	Part::Seq parts, partsNorm;
	/** Energy threshold */
	Real threshold;
	/** Energy normalisation constant */
	Real norm;

	/** Scale model scale */
	void setScale(Real scale) {
		partsNorm.resize(parts.size());

		for (size_t i = 0; i < parts.size(); i++) {
			const Part& src = parts[i];
			Part& dst = partsNorm[i];
			const Real s = src.imageScale ? scale : REAL_ONE;

			dst.numOfModels = src.numOfModels;
			dst.localPose.a = src.localPose.a;
			dst.localPose.p.v1 = s*src.localPose.p.v1;
			dst.localPose.p.v2 = s*src.localPose.p.v2;
			dst.linVar = s*src.linVar;
			dst.angVar = src.angVar;
			dst.threshold = src.threshold;
			dst.imageScale = src.imageScale;

			dst.init();
		}
	}

	/** Creates Model from the description. 
	* @param desc	Model description
	* @return		<code>TRUE</code> if no errors have occured;
	*				<code>FALSE</code> otherwise 
	*/
	bool create(const Model::Desc& desc);
	
	/** Releases resources */
	void release();
	
	Model(Layer& layer) : layer(layer) {
	}

public:
	/** Destructor is inaccesible */
	virtual  ~Model() {
	}

	/** Image layer */
	inline const Layer& getLayer() const {
		return layer;
	}

	/** Model signature */
	inline Signature::Type getSig() const {
		return sig;
	}
	
	/** Image filter */
	inline const ImageFilter2* getFilter() const {
		return filter.get();
	}
	
	/** Model parts */
	inline const Part::Seq& getParts() const {
		return partsNorm;
	}

	/** Energy threshold */
	inline Real getThreshold() const {
		return threshold;
	}

	/** Energy normalisation constant */
	inline Real getNorm() const {
		return norm;
	}
};

//------------------------------------------------------------------------------

/** Image part realisation */
class Realisation {
public:
	friend class Layer;

	/** Realisation sequence */
	typedef std::vector<Realisation> Seq;
	/** Realisation pointer sequence */
	typedef std::vector<const Realisation*> PtrSeq;
	/** Realisation rank */
	typedef golem::Rank<Real, Realisation, std::greater<Real> > Rank;

	/** Realisation part rank */
	typedef golem::ClassRank<Real, Signature::Type, const Realisation*, std::greater<Real> > ClassRank;
	/** Realisation part rank sequence */
	typedef std::vector<ClassRank> ClassRankSeq;

protected:
	/** Model equivalent */
	const Model* model;
	/** Pose */
	Pose2 pose;
	/** Realisation parts */
	Realisation::PtrSeq parts;
	/** Realisation signature */
	Signature::Type sig;
	/** Total energy */
	Real energy;
	
	/** Computes realisation signature */
	void computeSig() {
		ASSERT(model != NULL)

		if (model->getFilter() != NULL) {
			sig = (Signature::Type)model->getSig();
		}
		else {
			Signature::init(sig);
			
			for (Realisation::PtrSeq::const_iterator i = parts.begin(); i != parts.end(); i++) {
				Signature::Type data = (*i)->getSig();
				Signature::process(sig, (const U8*)&data, sizeof(Signature::Type));
			}

			Signature::finish(sig);
		}
	}

public:
	/** Model equivalent */
	inline const Model* getModel() const {
		return model;
	}
	
	/** Pose */
	inline const Pose2& getPose() const {
		return pose;
	}
	
	/** Realisation parts */
	inline const Realisation::PtrSeq &getParts() const {
		return parts;
	}
	
	/** Realisation signature */
	inline Signature::Type getSig() const {
		return sig;
	}
	
	/** Total energy */
	inline Real getEnergy() const {
		return energy;
	}
};

//------------------------------------------------------------------------------

/** Image layer */
class Layer : public Runnable {
public:
	/** Layer pointer type */
	typedef golem::shared_ptr<Layer> Ptr;
	/** Layer sequence type */
	typedef std::vector<Ptr> Seq;
	/** Layer data */
	typedef golem::Lattice2<I32, Realisation::Seq> RealisationArr;
	/** Models collection */
	typedef PrivateList<Model*, Model::Ptr> ModelList;
	/** Models identifiers */
	typedef std::map<Signature::Type, Model*> ModelMap;

	/** Layer description */
	class Desc {
	public:
		/** Layer description pointer */
		typedef golem::shared_ptr<Desc> Ptr;
		/** Layer description sequence */
		typedef std::vector<Ptr> Seq;
		
		/** Creates and initialises a set of layers, returns pointer to the first one. */
		virtual Layer::Ptr create(golem::Embodiment &embodiment, Layer* pLower) const {
			Layer::Ptr pLayer(new Layer(embodiment, pLower));

			if (!pLayer->create(*this))
				pLayer.release();

			return pLayer;
		}

		/** Layer scale reduction */
		Real scale;
		/** Number of model rotations */
		U32 numOfRotations;
		/** Maximum number of realisations per pixel */
		U32 numOfRealisations;
		/** Models descriptions */
		Model::Desc::Seq modelsDesc;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		
		virtual ~Desc() {
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			scale = Real(1.0/Math::sqrt(2.0)); // two per octave
			numOfRotations = 8;
			numOfRealisations = 10;
			modelsDesc.clear();
		}
		
		/** Add Gabor filter */
		void addGaborFilter(U32 numOfRotations, Real threshold) {
			modelsDesc.push_back(Model::Desc::Ptr(new Model::Desc));
			modelsDesc.back()->addGaborFilter(numOfRotations);
			modelsDesc.back()->threshold = threshold;
		}

		/** Adds 2 symmetric parts */
		void add2PartsSymm(U32 numOfRotations, Real dist, Real threshold) {
			for (U32 i = 1; i < numOfRotations; i++) {
				modelsDesc.push_back(Model::Desc::Ptr(new Model::Desc));
				modelsDesc.back()->add2PartsSymm(REAL_2_PI*Real(i)/Real(numOfRotations), dist);
				modelsDesc.back()->threshold = threshold;
			}
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (scale <= REAL_ZERO)
				return false;
			if (numOfRotations <= 0 || numOfRealisations <= 0)
				return false;
			for (Model::Desc::Seq::const_iterator i = modelsDesc.begin(); i != modelsDesc.end(); i++)
				if ((*i) == NULL || !(*i)->isValid())
					return false;

			return true;
		}
	};

protected:
	/** Data used by Parallels */
	class ParallelsData {
	public:
		RealisationArr::Index index;
		RealisationArr::Idx2 idx2;
		Vec2 position;
		
		Realisation::Rank realisationRank;
		Response::Seq responseSeq;

		Realisation::ClassRankSeq realisationClassRankSeq;
	};

	/** Data shared by all layers */
	class SharedData {
	public:
		/** Pointer */
		typedef golem::shared_ptr<SharedData> Ptr;
		
		/** Models signatures */
		ModelMap modelMap;
		/** Models counter */
		U32 modelCounter;
	};

	/** Embodiment */
	golem::Embodiment &embodiment;
	/** Context object */
	golem::Context &context;
	/** Layer index in hierarchy */
	U32 index;
	/** Layers in the hierarchy */
	Layer *pFirst, *pLower, *pUpper;
	/** Layer scale reduction */
	Real scale, scaleTotal;
	/** Maximum number of realisations per pixel */
	U32 numOfRealisations;
	/** Number of filter rotations */
	U32 numOfRotations;
	/** Rotation delta  */
	Real rotationDelta;
	
	/** Data shared by all layers */
	SharedData::Ptr sharedData;
	
	/** Models */
	ModelList modelList;
	/** Realisations */
	RealisationArr realisationArr;

	/** Processed image */
	const Image2* pImage;
	/** Parallels' critical section */
	CriticalSection parallelsCs;
	/** Parallels' image index */
	volatile RealisationArr::Index parallelsIndex;
	
	/** Model rotations */
	inline Real getRotation(U32 rotationIndex) const {
		return rotationDelta*rotationIndex;
	}
	
	/** Model rotations */
	inline Real getRotationInv(U32 rotationIndex) const {
		return -rotationDelta*rotationIndex;
	}

	/** Process current layer */
	virtual bool processLayer(const Image2* pImage);
	
	/** Process model */
	virtual bool processModel(const Model* pModel, ParallelsData& data) const;
	
	/** Process realisation */
	virtual bool processRealisation(U32 index, Real energy, Realisation& realisation, ParallelsData& data) const;
	
	/** Parallels' job. */
	virtual void run();

	/** Creates Layer from the description. 
	* @param desc	Layer description
	* @return		<code>TRUE</code> if no errors have occured;
	*				<code>FALSE</code> otherwise 
	*/
	bool create(const Layer::Desc& desc);
	
	/** ImageFilter constructor */
	Layer(golem::Embodiment &embodiment, Layer* pLower);
	
public:
	/** Destructor is inaccesible */
	virtual ~Layer();

	/** Creates a model in the layer */
	virtual Model* create(const Model::Desc& desc);

	/** Removes the model from the layer */
	virtual void remove(Model& model);
	
	/** Model collection */
	inline const ModelList& getModelList() const {
		return modelList;
	}

	/** Model map */
	inline const ModelMap& getModelMap() const {
		return sharedData->modelMap;
	}

	/** Process image on the layer hierarchy */
	virtual bool process(const Image2* pImage);

	/** Realisations array */
	inline const RealisationArr& getRealisationArr() const {
		return realisationArr;
	}

	/** Layer index in hierarchy */
	inline U32 getIndex() const {
		return index;
	}
	
	/** Tests for first layer */
	inline bool isFirst() const {
		return pLower == NULL;
	}
	
	/** Tests for last layer */
	inline bool isLast() const {
		return pUpper == NULL;
	}

	inline golem::Context& getContext() {
		return context;
	}
	inline const golem::Context& getContext() const {
		return context;
	}

	inline golem::Embodiment& getEmbodiment() {
		return embodiment;
	}
	inline const golem::Embodiment& getEmbodiment() const {
		return embodiment;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_LAYER_H_*/
