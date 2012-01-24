/**
 * @file VisionCore.hh
 * @author Zillich
 * @date March 2007
 * @version 0.1
 * @brief Core of all vision calculations
 **/

/**
 * TODO - for each principle and/or gestalt have a SetupParameters() function
 * TODO   called after getting the image
 * TODO - more gestalt principles, color!
 * TODO - anytimeness
 * TODO - feedback
 * TODO - different gestalt principles produce the same groups
 * TODO - animate forward/feedback processing
 * TODO - why all the static functions?
 * TODO - video should be created external to Visioncore (is often useful)
 */

#ifndef Z_VISION_CORE_HH
#define Z_VISION_CORE_HH

#include <opencv/cxcore.h>
#include <vector>
#include "Namespace.hh"
#include "Array.hh"
#include "Config.hh"
#include "Color.hh"
#include "Image.hh"
#include "Video.hh"
#include "Gestalt.hh"
#include "GestaltPrinciple.hh"

#include "CubeDefinition.hh"
#include "CylinderDefinition.hh"
#include "FlapDefinition.hh"

#include <mxCameraModel.h>

// #include "IceInterface.hh"
//#include "Tracker3D.hh"

namespace Z
{

/**
 * @brief Class to collect whatever statistics are interesting at the moment.
 * Basically one big continued hack.
 */
class Statistics
{
private:
  vector<double> time_series;
  vector<int> pix_cnt_series;
  vector<int> hop_cnt_series;
  vector<int> jcts_series;
  vector<int> clos_series;
  vector<int> attempt_path_search_series;
  vector<int> path_search_series;
  vector<int> path_max_visited_series;
  vector<double> path_avg_visited_series;
  vector<double> avg_clos_sig_series;

public:
  double time;
  int pix_cnt;
  int hop_cnt;
  int jcts;
  int clos;
  int attempt_path_search;
  int path_search;
  int path_max_visited;
  double path_avg_visited;
  int new_clos;
  double avg_clos_sig;

  Statistics();
  ~Statistics();
  void PushEntry();
};


/**
 * @brief Class representing the complete vision system.
 */
class VisionCore
{
private:
  static Image *img;													///< Images to process
  static Video *video;												///< Video to process

	static double camIntrinsic[4];							///< Intrinsic parameters of the cam
	static double camDistortion[4];							///< Distortion parameters of the cam
	static double camExtrinsic[12];							///< Extrinsic parameters of the cam

// 	static Tracker3D tracker;										///< Toms 3D tracker
//	static IceInterface *ice;										///< Ice-Interface to following systems

private:
	/**
	 *	@brief Initialise the defined Gestalt-Principles
	 */
  static void InitGestaltPrinciples();

public:
  static Array<Gestalt*> gestalts[];					///< all Gestalts
  static Array<unsigned> ranked_gestalts[];		///< all Ranked Gestalts
  static GestaltPrinciple* principles[];			///< all initialised Gestalt-Principles
  static double p_e;													///< probability of an edgel
  static double p_ee; 												///< probability of an edgel given another edgel
  static Config config;												///< Class to configure the vision system, by the configure-file
  static Statistics stats;										///< Class to collect statistic relevant data 

//  static Vector2 roi_center;	///< Hannes code
//  static double roi_sigma;		///< Hannes code
//  static IplImage *wmap;			///< Hannes code

public:

	/**
 	 *	@brief Constructor
	 *	@param vid_type Type of video
	 *	@param config_name Name of Configuration-File
	 *	@param cameraSource Number of camera source [/dev/video?]
 	 */
  VisionCore(VideoType vid_type, const char *config_name = 0, int cameraSource = 0);

	/**
 	 *	@brief Constructor
	 *	@param vid Video
	 *	@param config_name Name of Configuration-File
	 *	@param cameraSource Number of camera source [/dev/video?]
 	 */
  VisionCore(Video *vid, const char *config_name = 0, int cameraSource = 0);

	/**
	 *	@brief Destructor
	 */
  ~VisionCore();

	/**
	 * @brief Configure the vision system.
	 * @param config_name Name of Configuration-File
	 */
  void Configure(const char *config_name);

	/**
	 * @brief Enables a Gestalt-Principle
	 * @param p Type of Gestalt-Principle.
	 */
  static void EnableGestaltPrinciple(GestaltPrinciple::Type p);

	/**
	 * @brief Disables a Gestalt-Principle
	 * @param p Type of Gestalt-Principle.
	 */
  static void DisableGestaltPrinciple(GestaltPrinciple::Type p);

	/**
	 * @brief Get status of Gestalt-Principle
	 * @param p Type of Gestalt-Principle.
	 * @return True, if the Gestalt-Principle is enabled.
	 */	
  static bool IsEnabledGestaltPrinciple(GestaltPrinciple::Type p);

	/**
	 * @brief Fetch new image and clear all results (Gestalts) for processing the next one.
	 */
  static void NewImage();

	/**
	 * @brief Store new image and clear all results (Gestalts) for processing this one.
	 */
//   static void NewImage(Image *image);

	/**
	 * @brief Init buffer.
	 * @param img IplImage
	 * @param copy Set true, to copy data.
	 */
	void InitBuffer(IplImage *img, bool copy);

	/**
	 * @brief Set buffer.
	 * @param img IplImage
	 */
	void SetBuffer(IplImage *img);

	/**
	 * @brief Load new image
	 * @param filename Filename of the image to load.
	 */
  static void LoadImage(const char *filename);

	/**
	 * @brief Move to the first image of a file-video.
	 */
  static void FirstImage();

	/**
	 * @brief Move to the last image of a file-video.
	 */
  static void LastImage();

	/**
	 * @brief Move to the next image of a file-video.
	 * @return Returns true, if next image is available.
	 */
  static bool NextImage();

	/**
	 * @brief Move to the previous image of a file-video.
	 * @return Returns true, if previous image is available.
	 */
  static bool PrevImage();

	/**
	 * @brief Clear Images.
	 * TODO used?
	 */
  static void ClearImages();

	/**
	 * @brief Get image number.
	 * @return Number of the current image.
	 */
  static unsigned ImageNumber();

	/**
	 * @brief Get image name.
	 * @return Returns the name of the image.
	 */
  static const char *ImageName();

	/**
	 * @brief Get status of video.
	 * @return Returns true, if live-video is processed.
	 */
  static bool LiveVideo();

	/**
	 * @brief Get status of image.
	 * @return Returns true, if there is a image to process.
	 */
  static bool HaveImage() {return img != 0;}

	/**
	 * @brief Get width of the image.
	 * @return Returns the width of the image in pixel (integer).
	 */
  static int ImageWidth() {return video->Width();}

	/**
	 * @brief Get height of the image.
	 * @return Returns the height of the image in pixel (integer).
	 */
  static int ImageHeight() {return video->Height();}

	/**
	 * @brief Get width of the image.
	 * @return Returns the width of the image in pixel (unsigned).
	 */
  static unsigned IW() {return img->Width();}

	/**
	 * @brief Get height of the image.
	 * @return Returns the height of the image in pixel (integer).
	 */
  static unsigned IH() {return img->Height();}

	/**
	 * @brief Get the area of a image.
	 * @return Returns the area of the image in pixel.
	 */
  static int ImageArea() {return HaveImage() ? img->Width()*img->Height() : 0;}

	/**
	 * @brief Get the image in RGB24-values.
	 * @return Returns the image in RGB 24-Bit values, if available.
	 */
  static char* ImageRGB24() {return HaveImage() ? img->Data() : 0;}

	/**
	 * @brief Get the RGB-Color of a pixel from the image.
	 * @return Returns the RGB-Color of a pixel from the image.
	 */
  static RGBColor Pixel(int x, int y) {return *((RGBColor*)(img->Data(x, y)));}

	/**
	 * @brief Get the array of all Gestalts.
	 * @return Returns the Array with all Gestalts (Array<Gestalt*>).
	 */
  static Array<Gestalt*>* Gestalts() {return gestalts;}

	/**
	 *	@brief Inintialise the ice interface with Host and Port of the TCP/IP-Connection
	 *	@param host Host of the ice interface connection (default: localhost)
	 *	@param port Port of the ice interface connection (detault: 20000)
	 */
	static void IceInitialise(const char* host, const char* port);

	/**
	 * @brief Process the current image incrementally for a given amount of time and calculate 
	 * 				afterwards the non-incremental principles.
	 * @param runtime_ms  amount of time to run, in microseconds.
	 * @param ca Canny Alpha: sets canny alpha value
	 * @param co Canny Omega: sets canny omega value
	 * @param sendIce If true, Objects will be sent via ice interface
	 */  
	static void ProcessImage(int runtime_ms, int ca, int co, bool sendIce);

	/**
 	 * @brief Older, non-incremental (non-anytime) version of ProcessImage
 	 */
  static void ProcessImage();

	/**
 	 * @brief Clear all results 
	 * TODO: Unterschied zu ClearGestalts???
 	 */
  static void ClearResults();

	/**
 	 * @brief Clear all Gestalts
 	 */
  static void ClearGestalts();

	/**
	* @brief Draw the original image and all the gestalts we have.
	*/
  static void Draw(int detail = 0);

	/**
	* @brief Draw only the original image.Get the RGB-Color of a pixel from the image.
	*/
  static void DrawImage();

	/**
	* @brief Draw all the gestalts we have.
	* @param detail Degree of detail to draw Gestalt
	*/
  static void DrawAllGestalts(int detail = 0);

	/**
	* @brief Draw all gestalts of given type.
	* @param type Type of Gestalt
	* @param detail Degree of detail to draw Gestalt
	*/
  static void DrawGestalts(Gestalt::Type type, int detail = 0);

	/**
	* @brief Draw all gestalts of given type, which are unmasked.
	* @param type Type of Gestalt
	* @param detail Degree of detail to draw Gestalt
	*/
  static void DrawUnmaskedGestalts(Gestalt::Type type, int detail = 0);

	/**
	* @brief Draw just a single gestalt.
	* @param type Type of Gestalt
	* @param num Number of Gestalt (in order of existence ??? TODO)
	* @param detail Degree of detail to draw Gestalt
	*/
  static void DrawGestalt(Gestalt::Type type, unsigned num, int detail = 0);

	/**
	* @brief Draw whatever pictorial info a gestalt has to offer.
	* @param type Type of Gestalt
	* @param num Number of Gestalt (in order of existence ??? TODO)
	*/
  static void DrawGestaltInfo(Gestalt::Type type, unsigned num);

	/**
	* @brief Draw whatever the given gestalt principle has to draw.
	* @param type Type of Gestalt
	* @param detail Degree of detail to draw Gestalt
	*/
  static void DrawPrinciple(GestaltPrinciple::Type type, int detail = 0);

	/**
	* @brief Set the active draw area to the IplImage.
	* @param iI OpenCv IplImage for drawing.
	*/
	static void DrawArea(IplImage *iI);

	/**
	* @brief Draw all gestalts of given type (per integer).
	* @param type Type of Gestalt
	* @param detail Degree of detail to draw Gestalt
	*/
//   static int IplDrawGestalts(int type, int detail = 0);


	/**
	* @brief Returns id of first gestalt at pixel position (x,y).
	* start_after can be used to skip the first gestalts. So all gestalts at x,y
	* can be selected consecutively.
	* @param type Type of Gestalt
	* @param x X-coordinate of Gestalt
	* @param y Y-coordinate of Gestalt
	* @param start_at ??? TODO
	*/
  static unsigned PickGestaltAt(Gestalt::Type type, int x, int y, unsigned start_at);

	/**
	* @brief Accumulated runtime of active Gestalt principles.
	*/
  static double RunTime();

	/**
	 * @brief Set camera parameters
	 */
	static void SetCamParameters(double *intrinsic, double *distortion, double *extrinsic);

	/**
	 * @brief Get camera model
	 */
	static void GetCamModel(mx::CCameraModel &m_cCamModel);


	/// TODO TODO TODO TODO TODO TODO TODO TODO TODO 
	void GetObject(Gestalt::Type type, unsigned number, CubeDef cd);

	/**
	 * @brief Returns detected cube
	 * @param number Number of the cube
	 * @param cd Cube definition
	 * @param masked Returns true, if cube is masked
	 * @return Returns true, if cube with parameter number exists
	 **/
	bool GetCube(unsigned number, CubeDef &cd, bool &masked);

	/**
	 * @brief Returns detected cylinder
	 * @param number Number of the cylinder
	 * @param cd cylinder definition
	 * @param masked Returns true, if cylinder is masked
	 * @return Returns true, if cylinder with parameter number exists
	 **/
	bool GetCylinder(unsigned number, CylDef &cd, bool &masked);

	/**
	 * @brief Returns detected flap
	 * @param number Number of the flap
	 * @param cd Flap definition
	 * @param masked Returns true, if flap is masked
	 * @return Returns true, if flap with parameter number exists
	 **/
	bool GetFlap(unsigned number, FlapDef &fd, bool &masked);
};


/**
 * @brief Add a new Gestalt (of any type) to the system.
 * @param g  new Gestalt
 * @param inform  if true, inform other parts of the system of this new Gestalt,
 *                otherwise add quietly. Default is true.
 */
unsigned NewGestalt(Gestalt *g, bool inform = true);

/**
 * @brief Once a new Gestalt is created, inform all those interested.
 * TODO: let Principles register themselves here.
 */
void InformNewGestalt(Gestalt::Type type, unsigned id);

inline Array<Gestalt*>& Gestalts(Gestalt::Type type)
{
  return VisionCore::gestalts[type];
}

inline Gestalt* Gestalts(Gestalt::Type type, unsigned id)
{
  return VisionCore::gestalts[type][id];
}

inline Array<unsigned>& RankedGestalts(Gestalt::Type type)
{
  return VisionCore::ranked_gestalts[type];
}

inline unsigned RankedGestalts(Gestalt::Type type, unsigned id)
{
  return VisionCore::ranked_gestalts[type][id];
}

inline unsigned NumGestalts(Gestalt::Type type)
{
  return VisionCore::gestalts[type].Size();
}

inline GestaltPrinciple* Principles(GestaltPrinciple::Type type)
{
  return VisionCore::principles[type];
}

}

#endif
