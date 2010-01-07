
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include "tgEngine.h"

#ifndef _MODEL_ENTRY_
#define _MODEL_ENTRY_

class ModelEntry{
public:
	bool valid;
	bool bfc;
	bool textured;
	tgModel model;
	
	VisionData::VisualObjectPtr obj;
	cast::cdl::WorkingMemoryAddress castWMA;
		
	ModelEntry(){
		valid = false;
		bfc = true;
		textured = false;
	}
};

#endif
