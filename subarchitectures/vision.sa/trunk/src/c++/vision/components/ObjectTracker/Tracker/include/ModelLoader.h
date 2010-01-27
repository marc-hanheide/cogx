 /**
 * @file ModelLoader.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Loading geometrical models from file
 * @namespace Tracking
 */
 
#ifndef _MODELLOADER_H_
#define _MODELLOADER_H_

#include <stdlib.h>
#include <stddef.h>
#include <ply.h>

#include "PlyStructure.h"
#include "Model.h"

namespace Tracking{

/** @brief Loading geometrical models from file */
class ModelLoader
{
private:

	bool propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index);

public:
	ModelLoader();
	~ModelLoader();
	
	/** @brief Loads PLY (Polygon File Format, Stanford Triangle Format) 
	*		@param Model Storage for model
	*		@param filename path and filename of file to load
	*		@return true on success, false on failure
	*/
	bool LoadPly(Model &model, const char* filename);
	   
};

} // namespace Tracking

#endif
