

#ifndef _NURBS_TOOLS_H_
#define _NURBS_TOOLS_H_

#include <opennurbs.h>
#include <vector>
#include <v4r/TomGine/tgNurbsSurface.h>

class NurbsTools
{
public:
	static ON_NurbsSurface InitNurbsPatch(int order);
	static std::vector<double> GetElementVector(int dim, ON_NurbsSurface &surf);
	static void Refine(int dim, ON_NurbsSurface &surf);
	static  TomGine::tgNurbsSurfacePatch Convert(ON_NurbsSurface &on_surf);

private:

};

#endif // _NURBS_TOOLS_H_
