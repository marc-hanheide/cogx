
#ifndef __PLYMODEL_H__
#define __PLYMODEL_H__


#include <stdlib.h>
#include <stddef.h>
#include <ply.h>

#include "PlyStructure.h"
#include "Model.h"

class ModelLoader
{
private:

	bool propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index);

public:
	ModelLoader();
	~ModelLoader();
	
	//void write(const char* filename);
	bool LoadPly(Model &model, const char* filename);
   
};


#endif
