 /**
 * @file tgMaterial.h
 * @author Thomas Mörwald
 * @date January 2010
 * @version 0.1
 * @brief Specify material parameters for the OpenGL lighting model.
 */

#ifndef TG_MATERIAL_MODEL
#define TG_MATERIAL_MODEL

#include "tgMathlib.h"

namespace TomGine{

/** @brief Specify material parameters for the OpenGL lighting model. */
class tgMaterial
{
public:
	vec4 ambient;		///< ambient RGBA reflectance of the material
	vec4 diffuse;		///< diffuse RGBA reflectance of the material
	vec4 specular;		///< specular RGBA reflectance of the material
	vec4 color;			///< RGBA color (used directly if lighting disabled)
	float shininess;	///< RGBA specular exponent of the material

	/** @brief Create grey material. */
	tgMaterial();

	/** @brief Enables lighting and applies material to OpenGL. */
	void Activate() const;

	/** @brief Create material with a specific color reflectance RGBA */
	void Color(float r, float g, float b, float a=1.0f);

	/** @brief Create material with random color reflectance RGBA. */
	void Random();
};

} // namespace TomGine

#endif
