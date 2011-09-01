
#include <stdio.h>
#include "NurbsTools.h"

ON_NurbsSurface NurbsTools::InitNurbsPatch(int order){

	ON_NurbsSurface patch;

	patch = ON_NurbsSurface(3, false, order, order, order, order);
	patch.MakeClampedUniformKnotVector(0, 1.0);
	patch.MakeClampedUniformKnotVector(1, 1.0);

	double dcu = 1.0/(order-1);  // image sizes are power of two
	double dcv = 1.0/(order-1);

	ON_3dPoint cv(0.0,0.0,0.0);
	for(int i=0; i<order; i++) {
		for(int j=0; j<order; j++) {
			cv.x = -1.0 + 2.0*dcu * i;
			cv.y = -1.0 + 2.0*dcv * j;
			cv.z = 0.0;
			patch.SetCV(i,j,cv);
		}
	}

	return patch;
}

std::vector<double> NurbsTools::GetElementVector(int dim, ON_NurbsSurface &surf){
	std::vector<double> result;

	if(dim==0) {
		int idx_min = 0;
		int idx_max = surf.m_knot_capacity[0]-1;
		if(surf.IsClosed(0)){
			idx_min = surf.m_order[0]-2;
			idx_max = surf.m_knot_capacity[0]-surf.m_order[0]+1;
		}

		const double* knotsU = surf.Knot(0);

		result.push_back(knotsU[idx_min]);

		//for(int E=(m_surf->m_order[0]-2); E<(m_surf->m_knot_capacity[0]-m_surf->m_order[0]+2); E++) {
		for(int E=idx_min+1; E<=idx_max; E++) {

			if(knotsU[E]!=knotsU[E-1])	// do not count double knots
				result.push_back(knotsU[E]);

		}

	}
	else if(dim==1) {
		int idx_min = 0;
		int idx_max = surf.m_knot_capacity[1]-1;
		if(surf.IsClosed(1)){
			idx_min = surf.m_order[1]-2;
			idx_max = surf.m_knot_capacity[1]-surf.m_order[1]+1;
		}
		const double* knotsV = surf.Knot(1);

		result.push_back(knotsV[idx_min]);

		//for(int F=(m_surf->m_order[1]-2); F<(m_surf->m_knot_capacity[1]-m_surf->m_order[1]+2); F++) {
		for(int F=idx_min+1; F<=idx_max; F++) {

			if(knotsV[F]!=knotsV[F-1])
				result.push_back(knotsV[F]);

		}

	}
	else
		printf("ERROR: Index exceeds problem dimensions!\n");

	return result;
}

void NurbsTools::Refine(int dim, ON_NurbsSurface &surf){

	std::vector<double> xi;
	std::vector<double> elements = NurbsTools::GetElementVector(dim,surf);

	for(unsigned i=0; i<elements.size()-1; i++)
		xi.push_back( elements[i] + 0.5*(elements[i+1]-elements[i]) );

	for(unsigned i=0; i<xi.size(); i++)
		surf.InsertKnot(dim, xi[i], 1);
}

 TomGine::tgNurbsSurfacePatch NurbsTools::Convert(ON_NurbsSurface &on_surf){

	TomGine::tgNurbsSurfacePatch nurbsData;

	// copy order
	nurbsData.orderU = on_surf.m_order[0]-1;
	nurbsData.orderV = on_surf.m_order[0]-1;

	// copy knots
	if(on_surf.m_knot_capacity[0]<=1 || on_surf.m_knot_capacity[1]<=1){
		printf("[NurbsTools::Convert] Warning: ON knot vector empty.\n");
		return nurbsData;
	}

	nurbsData.knotsU.push_back(on_surf.Knot(0,0));
	for(int i=0; i<on_surf.m_knot_capacity[0]; i++)
		nurbsData.knotsU.push_back(on_surf.Knot(0,i));
	nurbsData.knotsU.push_back(on_surf.Knot(0,on_surf.m_knot_capacity[0]-1));

	nurbsData.knotsV.push_back(on_surf.Knot(1,0));
	for(int i=0; i<on_surf.m_knot_capacity[1]; i++)
		nurbsData.knotsV.push_back(on_surf.Knot(1,i));
	nurbsData.knotsV.push_back(on_surf.Knot(1,on_surf.m_knot_capacity[1]-1));

	// copy control points
	nurbsData.ncpsU = on_surf.m_cv_count[0];
	nurbsData.ncpsV = on_surf.m_cv_count[1];
	for(int j=0; j<on_surf.m_cv_count[1]; j++){
		for(int i=0; i<on_surf.m_cv_count[0]; i++){
			ON_4dPoint on_cp;
			TomGine::vec4 vcp;
			on_surf.GetCV(i,j,on_cp);
			vcp = TomGine::vec4(on_cp.x, on_cp.y, on_cp.z,on_cp.w);
			nurbsData.cps.push_back(vcp);
		}
	}
	return nurbsData;
}

