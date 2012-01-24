/**
 * @file Reasoner.cpp
 * @author Andreas Richtsfeld
 * @date August 2010
 * @version 0.1
 * @brief Reasoning about detected planes and detected visual objects.
 */

#include "Reasoner.h"


namespace Z
{

// Equality threshold for stereo Gestalts
static const double REA_EQUALITY = 0.03;

/**
 * @brief Constructor of reasoner class.
 */
Reasoner::Reasoner()
{
	havePlane = false; 		// have a detected dominant plane
	maxSG = 3;
	sgCounter = 0;
	gAF = false;
}

/**
 * @brief Process a new convex hull and decide, if it is a dominant plane for the scene.
 * @param pos Postion of center from convex hull.
 * @param rad Radius of convex hull
 * @param p Convex hull points
 */
void Reasoner::ProcessConvexHull(Vector3 pos, double rad, std::vector<Vector3> p)
{
	plane = new Z::Plane(pos, rad, p);
	havePlane = true;
}

/**
 * @brief Get the (dominant) plane as visual object.
 * @param obj Dominant plane (convex hull mesh) as visual object.
 */
bool Reasoner::GetPlane(VisionData::VisualObjectPtr &obj)
{
	if(havePlane)
	{
		plane->GetVisualObject(obj);
		return true;
	}
	else return false;
}

/**
 * @brief Process data of new frame
 * @param sc Stereo core
 */
bool Reasoner::Process(StereoCore *sc)
{
	score = sc;
	filteredObjs.Clear();
	objects.Clear();
	
	// Copy results from the stereo core
	for(unsigned i=0; i<StereoBase::MAX_TYPE; i++)
		stereoGestalts[i][sgCounter] = sc->GetStereoGestalts(i);
	
	if(gAF)		// if gestalt array is filled
	{
		// filter the objects
		FilterGestalt(StereoBase::STEREO_ELLIPSE);

		// store it internaly as objects
		CreateObjects();
	}

	// calculate stereo gestalt counter variable
	sgCounter++;
	if(sgCounter >= maxSG)
	{
		sgCounter = 0;
		gAF = true;
	}
	if(gAF) return true;
	return false;
}


/**																																			/// TODO Should be the same for all Gestalts
 * @brief Filter ellipses.
 */
void Reasoner::FilterGestalt(StereoBase::Type type)
{
	int i, j, k;
	i = sgCounter;
	j = sgCounter-1;
	k = sgCounter-2;
	if(j < 0) j += 3;
	if(k < 0) k += 3;

	StereoEllipses *ellipses_0 = (StereoEllipses*) stereoGestalts[type][i];		/// ellipses from type/frame t=0
	StereoEllipses *ellipses_1 = (StereoEllipses*) stereoGestalts[type][j];		/// ellipses from type/frame t=-1
	StereoEllipses *ellipses_2 = (StereoEllipses*) stereoGestalts[type][k];		/// ellipses from type/frame t=-2

	for(int l=0; l<ellipses_0->NumStereoMatches(); l++)								///  get every ellipse3d from frame 0
	{
		int bestMatch_1 = 0;
		int bestMatch_2 = 0;
		double bestComp_1 = HUGE;
		double bestComp_2 = HUGE;

		Ellipse3D ell0 = ellipses_0->Ellipses(l);
		
		/// compare now this ellipse with next frame
		for(int m=0; m<ellipses_1->NumStereoMatches(); m++)							///  get best ellipse3d from frame t-1
		{
			Ellipse3D ell1 = ellipses_1->Ellipses(m);
			double comp = ell0.Compare(ell1);
			if(comp < bestComp_1)
			{
				bestComp_1 = comp;
				bestMatch_1 = m;
			}
		}
// printf("Reasoner::FilterGestalt: best result: %u / %4.2f\n", bestMatch_1, bestComp_1);

		for(int n=0; n<ellipses_2->NumStereoMatches(); n++)							///  get best ellipse3d from frame t-2
		{
			Ellipse3D ell2 = ellipses_2->Ellipses(n);
			double comp = ell0.Compare(ell2);
			if(comp < bestComp_2)
			{
				bestComp_2 = comp;
				bestMatch_2 = n;
			}
		}
// printf("Reasoner::FilterGestalt: best result 2: %u / %4.2f\n", bestMatch_2, bestComp_2);

		// calculate overall equality-value
		double equality = (bestComp_1 * bestComp_2) * 100.;
		
// printf("=> Reasoner::FilterGestalt: equality: %4.2f", equality);
// if(equality < REA_EQUALITY) printf(" => GOOD\n");
// else printf("\n");

		// create object, if equality is under threshold
		if(equality < REA_EQUALITY)
		{
			VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
			score->GetVisualObject(type, l, obj);																				/// TODO SMOOTH the object, dependent to equality values
			filteredObjs.PushBack(obj);
		}
	}
}


/**
 * @brief This is a HACK filter, to show some good results.
 */
bool Reasoner::HackFilter(VisionData::VisualObjectPtr &obj)
{
	printf("Reasoner: The hack filter!\n");
	
	/// radius between 2-8 cm
	Vector3 v[3];
	v[0].x = obj->model->vertices[0].pos.x;
	v[0].y = obj->model->vertices[0].pos.y;
	v[0].z = obj->model->vertices[0].pos.z;
	v[1].x = obj->model->vertices[1].pos.x;
	v[1].y = obj->model->vertices[1].pos.y;
	v[1].z = obj->model->vertices[1].pos.z;
	v[2].x = obj->model->vertices[2].pos.x;
	v[2].y = obj->model->vertices[2].pos.y;
	v[2].z = obj->model->vertices[2].pos.z;
	
	if(Length(v[0]-v[1]) < 0.05 || Length(v[0]-v[1]) > 0.12)
	{
		printf("    => Höhe: FALSE: %4.4f\n", Length(v[0]-v[1]));
		return false;
	}
	else printf("    => Höhe TRUE: %4.4f\n", Length(v[0]-v[1]));
		
	if(Length(v[1]-v[2]) < 0.02 || Length(v[1]-v[2]) > 0.10)
	{
		printf("    => Radius: FALSE: %4.4f\n", Length(v[1]-v[2]));
		return false;
	}
	else printf("    => Radius TRUE: %4.4f\n", Length(v[1]-v[2]));
		
	return true;
}
	
/**
 * @brief Create objects from the filtered objects array.
 */
void Reasoner::CreateObjects()
{
	for(unsigned i=0; i<filteredObjs.Size(); i++)
	{	
		Vector3 pos;
		pos.x = filteredObjs[i]->pose.pos.x;
		pos.y = filteredObjs[i]->pose.pos.y;
		pos.z = filteredObjs[i]->pose.pos.z;
		Z::Object *o = new Z::Object(pos, filteredObjs[i]->model->vertices, filteredObjs[i]->model->faces);
		o->ProjectToPlane(plane);
		objects.PushBack(o);
	}
}

/**
 * @brief Get the results from the reasoner.
 * @param objs Visual object array
 */
void Reasoner::GetResults(Array<VisionData::VisualObjectPtr> &objs, bool unprojected)
{			
// printf("Reasoner::GetResults: objects: %u\n", objects.Size());
	for(unsigned i=0; i<objects.Size(); i++)
	{
		VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
		
		if(objects[i]->GetVisualObjectProjected(obj))		// GetVisualObject projected to ground plane
		{
			HackFilter(obj);															/// TODO TODO TODO We enabled the filter!
			objs.PushBack(obj);
		}
		else if(unprojected)
		{
			objects[i]->GetVisualObject(obj);
			objs.PushBack(obj);
		}
	}
	
// printf("Reasoner::GetResults: objs: %u\n", objs.Size());
	
// 	objs = filteredObjs;
}




}




