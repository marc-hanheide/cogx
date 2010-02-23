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

#include "tgMathlib.h"
#include "tgPose.h"
#include "tgModel.h"

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
		float shininess;
		
		void Apply();
	};
	
	tgPose			m_pose;
	Material 		m_material;
	
	virtual void DrawFaces();
	virtual void DrawNormals(float normal_length);

private:
	void ApplyMaterial(Material mat);

};

} // namespace TomGine

#endif
