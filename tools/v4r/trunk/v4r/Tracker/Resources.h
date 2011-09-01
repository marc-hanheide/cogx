
#ifndef __RESOURCES_H__
#define __RESOURCES_H__

#include "headers.h"
#include "TrackerModel.h"
#include "ModelLoader.h"
#include <v4r/TomGine/tgSingleton.h>
#include <v4r/TomGine/tgShader.h>
#include <v4r/TomGine/tgImageProcessor.h>
#include <opencv/cv.h>



#ifndef FN_LEN
#define FN_LEN 256
#endif

#define g_Resources Resources::GetInstance()

namespace Tracking{

typedef std::vector<TomGine::tgShader*> ShaderList;
typedef std::vector<char*> NameList;


class Resources : public TomGine::tgSingleton <Resources>
{
friend class TomGine::tgSingleton <Resources>;
private:
	Resources();
	
	// Singleton Resources (one instance in programm)
	CvCapture* 			m_capture;
	IplImage* 			m_image;

	TomGine::tgImageProcessor *m_ip;
	
	// Resources lists
	ShaderList		m_shaderList;
	
	// Name lists
	NameList		m_shaderNameList;
	
	string			m_shaderPath;
	
	bool			m_showlog;
	
	int				SearchName(NameList* list, const char* filename);

public:
	~Resources();
	static Resources* GetInstance(){
		return TomGine::tgSingleton <Resources>::GetInstance ();
	}
	
	// Initialisation
	IplImage*				InitCapture(const char* file);

	IplImage* 				InitCapture(float width=640.0, float height=480.0, int camID = CV_CAP_ANY);

	TomGine::tgImageProcessor* InitImageProcessor(int width, int height);

    // Release-functions
	void ReleaseCapture();
	void ReleaseImageProcessor();
    
    // Set-function
	void 	SetShaderPath(const char* path){ m_shaderPath = path; }
	void	ShowLog(bool b){ m_showlog = b; }

	// Get-functions
	IplImage* 			GetNewImage();
	IplImage* 			GetImage();
	TomGine::tgImageProcessor* 	GetImageProcessor();
	
	TomGine::tgShader* GetShader(int id);
	
	// Add-functions
	int		AddShader(	const char* shadername,
						const char* vertex_file = NULL,
						const char* fragment_file = NULL,
						const char* header = NULL);
	
	// Release-functions
	void ReleaseShader();
	void ReleaseShader(int id);
	
	// Search-functions
	int	SearchShaderName(const char* filename);
};

} // namespace Tracking

#endif
