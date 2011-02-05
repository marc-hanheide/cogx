 /**
 * @file ModelEntry.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Defining the structure for a model containing the WorkingMemory Entry and the local model for rendering.
 */
 
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include "tgEngine.h"
#include "tgLabel.h"

#ifndef _MODEL_ENTRY_
#define _MODEL_ENTRY_

class ModelEntry{
public:
	bool valid;
	bool bfc;
	bool textured;
	
	TomGine::tgRenderModel model;
	TomGine::tgLabel label;
	
	cast::cdl::WorkingMemoryAddress castWMA;
		
	ModelEntry(){
		valid = false;
		bfc = true;
		textured = false;
	}
	
	ModelEntry(const char* fontfile){
		valid = false;
		bfc = true;
		textured = false;
		label = TomGine::tgLabel(fontfile);
	}

};

#endif
