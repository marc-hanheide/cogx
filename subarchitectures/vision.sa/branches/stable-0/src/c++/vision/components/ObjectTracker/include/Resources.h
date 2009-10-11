
#ifndef __RESOURCES_H__
#define __RESOURCES_H__

#include "Singleton.h"
#include <vector>
#include <stdio.h>
#include <opencv/highgui.h>
#include <SDL/SDL.h>

#include "ImageProcessor.h"
#include "Model.h"
#include "PlyModel.h"
#include "Texture.h"
#include "Shader.h"
#include "Camera.h"
#include "Particles.h"
#include "Frustum.h"

#ifndef FN_LEN
#define FN_LEN 256
#endif

#define g_Resources Resources::GetInstance()

typedef std::vector<Model*> ModelList;
typedef std::vector<Texture*> TextureList;
typedef std::vector<Shader*> ShaderList;
typedef std::vector<Camera*> CameraList;
typedef std::vector<char*> NameList;

class Resources : public Singleton <Resources>
{
friend class Singleton <Resources>;
private:
	Resources();
	
	// Singleton Resources (one instance in programm)
	CvCapture* 			m_capture;
	IplImage* 			m_image;
	SDL_Surface* 		m_screen;
	ImageProcessor* 	m_ip;
	Particles*			m_particles;
	Frustum*			m_frustum;
	
	// Resources lists
	ModelList		m_modelList;
	TextureList		m_textureList;
	ShaderList		m_shaderList;
	CameraList		m_cameraList;
	
	// Name lists
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
    IplImage* 		InitCapture(float width=320.0, float height=240.0, int camID = CV_CAP_ANY);
    SDL_Surface* 	InitScreen(int width, int height);
    ImageProcessor*	InitImageProcessor(int width, int height, Camera* cam);
    Particles*		InitParticles(int num, Particle p);
    Frustum*		InitFrustum();
    
    // Release-functions
	void ReleaseScreen();
	void ReleaseCapture();
	void ReleaseImageProcessor();
	void ReleaseParticles();
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
    Particles*		GetParticles();
    Frustum*		GetFrustum();
    
    Model*		GetModel(int id){ return m_modelList[id]; }
    Texture*	GetTexture(int id){ return m_textureList[id]; }
    Shader*		GetShader(int id){ return m_shaderList[id]; }
    Camera*		GetCamera(int id){ return m_cameraList[id]; }
    
    int		GetNumModels(){ return m_modelList.size(); }
    int		GetNumTextures(){ return m_textureList.size(); }

    // Add-functions
    int		AddModel(Model* model, const char* name);
	int		AddPlyModel(const char* filename);
	int		AddTexture(const char* filename, const char* texturename = NULL);
	int		AddShader(	const char* shadername,
						const char* vertex_file = NULL,
						const char* fragment_file = NULL,
						const char* header = NULL);
	int		AddCamera(const char* name);
	
	
	
	void ReleaseModel();
	void ReleaseTexture();
	void ReleaseShader();
	void ReleaseCamera();
	
	// Search-functions
    int		SearchModelName(const char* filename);
	int		SearchTextureName(const char* filename);
	int		SearchShaderName(const char* filename);
	int		SearchCameraName(const char* name);	
};

#endif
