 /**
 * @file tgRenderModel.h
 * @author Thomas Mörwald
 * @date January 2010
 * @version 0.1
 * @brief Defining a model for rendering.
 */

#ifndef TG_RENDER_MODEL
#define TG_RENDER_MODEL

#include <stdio.h>
#include <vector>

#include "TomGine/tgMathlib.h"
#include "TomGine/tgPose.h"
#include "TomGine/tgModel.h"
#include "TomGine/tgLabel.h"
#include "TomGine/tgMaterial.h"

namespace TomGine{



/** @brief Class tgRenderModel */
class tgRenderModel : public tgModel
{	
public:
	tgPose			m_pose;
	tgMaterial 		m_material;
	tgModel*		m_bsmodel;	// bounding sphere
	
	tgRenderModel();
	tgRenderModel(const tgModel& model);
	~tgRenderModel();
	
	void ApplyMaterial();
	void ApplyColor();
	
	virtual void DrawFaces();
	void DrawFaces(bool lighting);
// 	virtual void DrawPolygons();
	virtual void DrawNormals(float normal_length);
	
	/** @brief draws bounding sphere of model */
	virtual void DrawBoundingSphere();
};

} // namespace TomGine

#endif
