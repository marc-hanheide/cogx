/** @file Recognition.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_RECOGNITION_H_
#define _GOLEM_DEMO_COMMON_RECOGNITION_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Stream.h>
#include <Golem/Phys/Renderer.h>
#include <Golem/Demo/Common/ImageFilter.h>
#include <Golem/Demo/Common/Embodiment.h>
#include <Golem/Demo/Common/Layer.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Recognition input data */
struct RecognitionInp {
	/** image */
	const Image3* image;
};

/** Recognition output data */
struct RecognitionOut {
	/** top-layer realisations energy sorted */
	Realisation::PtrSeq realisations;
};

//------------------------------------------------------------------------------

/** Renders realisations */
class RealisationRenderer : public DebugRenderer {
public:
	typedef shared_ptr<RealisationRenderer> Ptr;

	/** Part renderer description */
	class Desc {
	public:
		/** realisations to render */
		const Realisation::PtrSeq* realisations;
		/** global pose of the 2D image layers */
		Mat34 pose;
		
		/** number of top realisations to render */
		U32 numOfRealisations;
		/** layer pose shift */
		Real shift;
		/** realisation rotation length */
		Real rotation;
		/** level depth */
		U32 depth;

		/** Composition node colour */
		RGBA nodeColour;
		/** Composition rotation colour */
		RGBA rotationColour;
		/** Composition edge colour */
		RGBA edgeColour;
		/** Composition selected edge colour */
		RGBA edgeSelectColour;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			realisations = NULL;
			pose.setId();
			
			numOfRealisations = 10;
			shift = Real(0.025);
			rotation = Real(0.005);
			depth = 10;

			nodeColour = RGBA::WHITE;
			rotationColour = RGBA::BLUE;
			edgeColour.set(213, 140, 43, 70); // violetish
			edgeSelectColour.set(43, 212, 120, 255); // yellowish
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (realisations != NULL) {
				for (Realisation::PtrSeq::const_iterator i = realisations->begin(); i < realisations->end(); i++) {
					if (*i == NULL)
						return false;
				}
			}

			if (!pose.isFinite() || numOfRealisations <= 0 || depth < 1 || shift <= REAL_ZERO || rotation <= REAL_ZERO)
				return false;
			
			return true;
		}
	};

	bool create(const Desc &desc);

	void addRealisation(const Realisation* pUpper, const Realisation* pLower, const Desc& desc, U32 depth);
};

//------------------------------------------------------------------------------

/** Recognition
*/
class Recognition : public Filter<RecognitionInp, RecognitionOut> {
public:
	typedef shared_ptr<Recognition> Ptr;
	typedef Filter<RecognitionInp, RecognitionOut> Base;

	/** Object description */
	class Desc : public Base::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(Recognition, Channel::Ptr, Embodiment&)

	public:
		/** Layers options */
		Layer::Desc::Seq layersDesc;
		/** Part renderer description */
		RealisationRenderer::Desc realisationRendererDesc;
		/** Sorting */
		bool sort;
		
		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Base::Desc::setToDefault();

			name = "Recognition";

			layersDesc.clear();
			//layersDesc.push_back(Layer::Desc::Ptr(new Layer::Desc));
			
			realisationRendererDesc.setToDefault();
			sort = true;
		}
		
		/** Adds default layer */
		void addDefaultLayer() {
			layersDesc.push_back(Layer::Desc::Ptr(new Layer::Desc));
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Base::Desc::isValid())
				return false;

			if (layersDesc.empty())
				return false;
			for (Layer::Desc::Seq::const_iterator i = layersDesc.begin(); i != layersDesc.end(); i++)
				if (*i == NULL || !(*i)->isValid())
					return false;
			
			if (!realisationRendererDesc.isValid())
				return false;

			return true;
		}
	};

protected:
	/** Realisation energy comparator */
	struct RealisationCmpr : public std::binary_function<const Realisation*, const Realisation*, bool> {
		inline bool operator () (const Realisation* l, const Realisation* r) const {
			return l->getEnergy() > r->getEnergy();
		}
	};
	
	/** Generator of pseudo random numbers */
	Rand rand;
	
	/** Layers */
	Layer::Seq layers;

	/** Realisation renderer */
	RealisationRenderer realisationRenderer;
	RealisationRenderer::Desc realisationRendererDesc;
	/** Sorting */
	bool sort;

	/** image */
	Image2 image;
	
	CriticalSection cs;

	/** Renders the object. */
	virtual void render();
	
	/** Keyboard handler. */
	virtual void keyboardHandler(unsigned char key, int x, int y);
	
	/** Creates object from description. */
	bool create(const Recognition::Desc& desc);
	
	/** Constructor */
	Recognition(Embodiment &embodiment);

public:
	/** Destructor is inaccesible */
	virtual ~Recognition();

	/** Process data */
	virtual bool process(RecognitionOut &out, const RecognitionInp &inp, MSecTmU32 timeOut = MSEC_TM_U32_INF);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_RECOGNITION_H_*/
