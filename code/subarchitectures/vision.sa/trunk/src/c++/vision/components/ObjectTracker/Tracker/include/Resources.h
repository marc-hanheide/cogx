
#ifndef __RESOURCES_H__
#define __RESOURCES_H__

#include "headers.h"
#include "Singleton.h"
#include "ImageProcessor.h"
#include "TrackerModel.h"
#include "ModelLoader.h"
#include "Texture.h"
#include "Shader.h"
#include "Camera.h"
#include "Particles.h"
#include "Frustum.h"

#ifndef FN_LEN
#define FN_LEN 256
#endif

#define g_Resources Resources::GetInstance()


typedef std::vector<Particles*> ParticlesList;
typedef std::vector<TrackerModel*> ModelList;
typedef std::vector<Texture*> TextureList;
typedef std::vector<Shader*> ShaderList;
typedef std::vector<Camera*> CameraList;
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
	Frustum*				m_frustum;
	ModelLoader			m_modelloader;
	
	// Resources lists
	ParticlesList	m_particlesList;
	ModelList			m_modelList;
	TextureList		m_textureList;
	ShaderList		m_shaderList;
	CameraList		m_cameraList;
	
	// Name lists
	NameList		m_particlesNameList;
	NameList 		m_modelNameList;
	NameList		m_textureNameList;
	NameList		m_shaderNameList;
	NameList		m_cameraNameList;
	
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
    IplImage* 				InitCapture(float width=320.0, float height=240.0, int camID = CV_CAP_ANY);
    SDL_Surface* 			InitScreen(int width, int height, const char* name="SDL Window");
    ImageProcessor*		InitImageProcessor(int width, int height);
    Frustum*					InitFrustum();
    
    // Release-functions
	void ReleaseScreen();
	void ReleaseCapture();
	void ReleaseImageProcessor();
	void ReleaseFrustum();
    
    // Set-function
	void	SetModelPath(const char* path){ sprintf(m_modelPath, "%s", path); }
	void 	SetTexturePath(const char* path){ sprintf(m_texturePath, "%s", path); }
	void 	SetShaderPath(const char* path){ sprintf(m_shaderPath, "%s", path); }
	void	ShowLog(bool b){ m_showlog = b; }

    // Get-functions
    IplImage* 		GetNewImage();
    IplImage* 		GetImage();
    SDL_Surface* 	GetScreen();
    ImageProcessor* GetImageProcessor();
    Frustum*		GetFrustum();
    
    Particles*	GetParticles(int id){ return m_particlesList[id]; }
    TrackerModel*		GetModel(int id){ return m_modelList[id]; }
    Texture*	GetTexture(int id){ return m_textureList[id]; }
    Shader*		GetShader(int id){ return m_shaderList[id]; }
    Camera*		GetCamera(int id){ return m_cameraList[id]; }
    
    int		GetNumTracker(){ return m_tracker_id; }
    int		GetNumParticles(){ return m_particlesList.size(); }
    int		GetNumModels(){ return m_modelList.size(); }
    int		GetNumTextures(){ return m_textureList.size(); }

    // Add-functions
    int		AddParticles(int num, Particle p, const char* name);
    int		AddModel(TrackerModel* model, const char* name);
	int		AddPlyModel(const char* filename);
	int		AddTexture(const char* filename, const char* texturename = NULL);
	int		AddShader(	const char* shadername,
						const char* vertex_file = NULL,
						const char* fragment_file = NULL,
						const char* header = NULL);
	int		AddCamera(const char* name);
	void	AddTracker(){ m_tracker_id++; }
	
	// Release-functions
	void ReleaseParticles();
	void ReleaseModel();
	void ReleaseTexture();
	void ReleaseShader();
	void ReleaseCamera();
	
	// Search-functions
	int	SearchParticlesName(const char* name);
    int	SearchModelName(const char* filename);
	int	SearchTextureName(const char* filename);
	int	SearchShaderName(const char* filename);
	int	SearchCameraName(const char* name);	
};

#endif
