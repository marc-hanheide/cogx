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
// #include <GL/glu.h>

#include "tgMathlib.h"
#include "tgPose.h"
#include "tgModel.h"
#include "tgLabel.h"

namespace TomGine{

/**
* @brief Class tgRenderModel
*/
class tgRenderModel : public tgModel
{	
public:
	
	struct Material{
		vec4 ambient;
		vec4 diffuse;
		vec4 specular;
		vec4 color;
		float shininess;
		void Apply();
		Material();
	};
	
	tgPose			m_pose;
	Material 		m_material;
	
	void ApplyMaterial();
	void ApplyColor();
	
	virtual void DrawFaces();
	void DrawFaces(bool lighting);
	virtual void DrawPolygons();
	virtual void DrawNormals(float normal_length);



};

} // namespace TomGine

#endif
