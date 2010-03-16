 /**
 * @file tgModelLoader.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Loading geometrical models from file
 * @namespace Tracking
 */
 
#ifndef TG_MODELLOADER
#define TG_MODELLOADER

#include <stdlib.h>
#include <stddef.h>
#include <ply.h>
#include <string>

#include "PlyStructure.h"
#include "tgModel.h"
#include "tgRenderModel.h"


namespace TomGine{

/** @brief Loading geometrical models from file */
class tgModelLoader
{
private:

	bool propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index);

public:
	tgModelLoader();
	~tgModelLoader();
	
	/** @brief Loads PLY (Polygon File Format, Stanford Triangle Format) 
	*		@param Model Storage for model
	*		@param filename path and filename of file to load
	*		@return true on success, false on failure
	*/
	bool LoadPly(tgModel &model, const char* filename);
	
	bool SavePly(tgModel &model, const char* filename);
	   
};

} // namespace TomGine

#endif
