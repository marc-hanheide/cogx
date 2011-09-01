#include "NurbsCreator.h"
#include "NurbsTools.h"
#include "PopoutProcessor.h"

#include <v4r/TomGine/tgTomGineThread.h>

using namespace TomGine;

void updateTomGine(tgTomGineThread *tgThread,
		TomGine::tgNurbsSurfacePatch &surf, int surf_id,
		ON_NurbsSurface &on_surf)
{
	surf = NurbsTools::Convert(on_surf);

	surf.resU = 64;
	surf.resV = 64;
	if (surf_id < 0)
		surf_id = tgThread->AddNurbsSurface(surf);
	else
		tgThread->SetNurbsSurface(surf_id, surf);
	vec4 cor = vec4(0.0, 0.0, 0.0, 0.0);
	for (unsigned i = 0; i < surf.cps.size(); i++)
	{
		cor = cor + surf.cps[i];
	}
	cor = cor / float(surf.cps.size());
	cv::Vec3d cv_cor = cv::Vec3d(cor.x, cor.y, cor.z);
	tgThread->SetRotationCenter(cv_cor);
}

int main()
{
	PopoutProcessor* popout = new PopoutProcessor();
	PopoutProcessor::SharedData data;

	cv::Ptr < TomGine::tgTomGineThread > dbgWin;
	TomGine::tgNurbsSurfacePatch surf;
	int surf_id = -1;
	int pcl_id = -1;

	unsigned order = 2;
	unsigned refinement = 2;
	unsigned iterations = 5;

	while (!popout->Stopped())
	{

		printf("[main] WaitForData ...\n");
		popout->WaitForData(data);
		printf("[main] got data\n");

		NurbsCreator::FitNurbsSurface(order, refinement, iterations,
				data.matCloud, data.mask, data.contour);

		// Set up TomGine
		if (dbgWin.empty())
		{
			dbgWin = new TomGine::tgTomGineThread(data.image.cols,
					data.image.rows);
			cv::Mat_<float> camR = cv::Mat::eye(3, 3, CV_32F);
			cv::Mat camT = cv::Mat::zeros(3, 1, CV_32F);
			cv::Mat camI(cv::Mat::zeros(3, 3, CV_64F));
			camI.at<double> (0, 0) = camI.at<double> (1, 1) = 525;
			camI.at<double> (0, 2) = 320;
			camI.at<double> (1, 2) = 240;
			camI.at<double> (2, 2) = 1;
			dbgWin->SetCamera(camI);
			dbgWin->SetCamera(camR, camT);
		}

		if (pcl_id >= 0)
			dbgWin->SetPointCloud(pcl_id, data.matCloud);
		else
			pcl_id = dbgWin->AddPointCloud(data.matCloud);

//		updateTomGine(dbgWin, surf, surf_id, nurbs);

		dbgWin->Update();

		usleep(100000);
	}

	delete (popout);

	return 0;
}

