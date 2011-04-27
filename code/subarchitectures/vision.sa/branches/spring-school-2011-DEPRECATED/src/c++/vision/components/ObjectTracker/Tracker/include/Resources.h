
#ifndef __RESOURCES_H__
#define __RESOURCES_H__

#include "headers.h"
#include "Singleton.h"
#include "ImageProcessor.h"
#include "TrackerModel.h"
#include "ModelLoader.h"
#include "Texture.h"
#include "Shader.h"

#ifndef FN_LEN
#define FN_LEN 256
#endif

#define g_Resources Resources::GetInstance()

namespace Tracking{

typedef std::vector<TrackerModel*> ModelList;
typedef std::vector<Texture*> TextureList;
typedef std::vector<Shader*> ShaderList;
typedef std::vector<char*> NameList;


class Resources : public Singleton <Resources>
{
friend class Singleton <Resources>;
private:
	Resources();
	
	int m_tracker_id;
	
	// Singleton Resources (one instance in programm)
	CvCapture* 			m_capture;
	IplImage* 			m_image;
	SDL_Surface* 		m_screen;
	ImageProcessor* m_ip;
	ModelLoader			m_modelloader;
	
	// Resources lists
	ModelList			m_modelList;
	TextureList		m_textureList;
	ShaderList		m_shaderList;
	
	// Name lists
	NameList		m_particlesNameList;
	NameList 		m_modelNameList;
	NameList		m_textureNameList;
	NameList		m_shaderNameList;
	
	char			m_modelPath[FN_LEN];
	char			m_texturePath[FN_LEN];
	char			m_shaderPath[FN_LEN];
	
	bool			m_showlog;
	
	int				SearchName(NameList* list, const char* filename);

public:
	~Resources();
	static Resources* GetInstance(){
		return Singleton <Resources>::GetInstance ();
	}
	
	// Initialisation
	IplImage*					InitCapture(const char* file);
	IplImage* 				InitCapture(float width=640.0, float height=480.0, int camID = CV_CAP_ANY);
	SDL_Surface* 			InitScreen(int width, int height, const char* name="SDL Window");
	ImageProcessor*		InitImageProcessor(int width, int height);

    // Release-functions
	void ReleaseScreen();
	void ReleaseCapture();
	void ReleaseImageProcessor();
    
    // Set-function
	void	SetModelPath(const char* path){ sprintf(m_modelPath, "%s", path); }
	void 	SetTexturePath(const char* path){ sprintf(m_texturePath, "%s", path); }
	void 	SetShaderPath(const char* path){ sprintf(m_shaderPath, "%s", path); }
	void	ShowLog(bool b){ m_showlog = b; }

	// Get-functions
	IplImage* 			GetNewImage();
	IplImage* 			GetImage();
	SDL_Surface* 		GetScreen();
	ImageProcessor* GetImageProcessor();
	
	TrackerModel*		GetModel(int id);
	Texture*				GetTexture(int id);
	Shader*					GetShader(int id);
	
	int		GetNumTracker(){ return m_tracker_id; }
	int		GetNumModels(){ return m_modelList.size(); }
	int		GetNumTextures(){ return m_textureList.size(); }

	// Add-functions
	int		AddModel(TrackerModel* model, const char* name);
	int		AddPlyModel(const char* filename);
	int		AddTexture(const char* filename, const char* texturename = NULL);
	int		AddTexture(Texture* texture);
	int		AddShader(	const char* shadername,
						const char* vertex_file = NULL,
						const char* fragment_file = NULL,
						const char* header = NULL);
	void	AddTracker(){ m_tracker_id++; }
	
	// Release-functions
	void ReleaseModel();
	void ReleaseTexture();
	void ReleaseShader();
	void ReleaseShader(int id);
	
	// Search-functions
	int	SearchModelName(const char* filename);
	int	SearchTextureName(const char* filename);
	int	SearchShaderName(const char* filename);
};

} // namespace Tracking

#endif
