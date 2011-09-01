#include <igal/PatchFitting.h>
#include <igal/DataLoading.h>
#include "NurbsCreator.h"

void updateTomGine(TomGine::tgTomGineThread *dbgWin, ON_NurbsSurface on_surf)
{
	static int surf_id = -1;
	TomGine::tgNurbsSurfacePatch surf = NurbsTools::Convert(on_surf);

	surf.resU = 64;
	surf.resV = 64;
	if (surf_id < 0)
		surf_id = dbgWin->AddNurbsSurface(surf);
	else
		dbgWin->SetNurbsSurface(surf_id, surf);
	TomGine::vec4 cor = TomGine::vec4(0.0, 0.0, 0.0, 0.0);
	for (unsigned i = 0; i < surf.cps.size(); i++)
	{
		cor = cor + surf.cps[i];
	}
	cor = cor / float(surf.cps.size());
	cv::Vec3d cv_cor = cv::Vec3d(cor.x, cor.y, cor.z);
	dbgWin->SetRotationCenter(cv_cor);
	dbgWin->Update();
}

void NurbsCreator::FitNurbsSurface(unsigned order, unsigned refinement,
		unsigned iterations, cv::Mat_<cv::Vec4f> matCloud,
		cv::Mat_<uchar> mask, cv::Mat_<uchar> contour,
		TomGine::tgTomGineThread* dbgWin)
{
	printf("[NurbsCreator::FitNurbsSurface] fitting NURBS surface ...\n");

	DataManager data_set;
	data_set.FromCvMat2DataSet(matCloud, mask, contour);

	PatchFitting patchFit(order, data_set[0]);

	static int int_id = -1, bnd_id = -1;
	TomGine::tgModel tgm_interior, tgm_boundary;
	for (int i = 0; i < data_set[0]->interior.PointCount(); i++)
	{
		TomGine::tgColorPoint cpt;
		cpt.pos = TomGine::vec3(
				data_set[0]->interior[i].x,
				data_set[0]->interior[i].y,
				data_set[0]->interior[i].z);
		cpt.color =
		{	0, 0, 255};
		tgm_interior.m_colorpoints.push_back(cpt);
	}
	for(int i=0; i<data_set[0]->boundary.PointCount(); i++)
	{
		TomGine::tgColorPoint cpt;
		cpt.pos = TomGine::vec3(
				data_set[0]->boundary[i].x,
				data_set[0]->boundary[i].y,
				data_set[0]->boundary[i].z);
		cpt.color = {255, 0, 0};
		tgm_boundary.m_colorpoints.push_back(cpt);
	}
	if(int_id<0)
		int_id = dbgWin->AddPointCloud(tgm_interior);
	else
		dbgWin->SetPointCloud(int_id, tgm_interior);
	if(bnd_id<0)
		bnd_id = dbgWin->AddPointCloud(tgm_boundary);
	else
		dbgWin->SetPointCloud(bnd_id, tgm_boundary);

	updateTomGine(dbgWin, *patchFit.m_patch);
	usleep(200000);

	for (unsigned r = 0; r < refinement; r++)
	{
		printf("[NurbsCreator::FitNurbsSurface]refinement: %d\n", r);

		patchFit.assemble(	1, 1,
							100.0, 0.0,
							0.0, 0.0,
							50.0+r*100.0, 1.0,
							0.0);

//				void assemble(	int resU, int resV,
//					double wBnd, double wInt,
//					double wCurBnd, double wCurInt,
//					double wCageRegBnd, double wCageReg,
//					double wCorner);

		patchFit.solve();
		updateTomGine(dbgWin, *patchFit.m_patch);

		patchFit.refine(0);
		patchFit.refine(1);
	}
	for (int i = 0; i < iterations; i++)
	{
		patchFit.assemble(	1, 1,
							100.0, 1.0,
							0.0, 0.0,
							200.0, 1.0,
							0.0);
		patchFit.solve();
		updateTomGine(dbgWin, *patchFit.m_patch);
	}

}
