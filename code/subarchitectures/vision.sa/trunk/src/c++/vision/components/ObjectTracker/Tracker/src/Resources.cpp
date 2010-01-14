
#include "Resources.h"

// *** PRIVATE ***

int Resources::SearchName(NameList* list, const char* filename){
	// parse through name list
	int i = 0;
	NameList::iterator it_name = list->begin();
	while(it_name != list->end()){
		if(!strcmp((*it_name),filename))
			return i;
		it_name++;
		i++;
	}
	
	// not found
	return -1;
}

// *** PUBLIC ***

Resources::Resources(){
	m_capture = 0;
	m_image = 0;
	m_screen = 0;
	m_ip = 0;
	m_frustum = 0;
	m_showlog = false;
	m_tracker_id = 0;
}

Resources::~Resources(){
	ReleaseCapture();
	ReleaseScreen();
	ReleaseImageProcessor();
	ReleaseFrustum();
	
	
	ReleaseParticles();
	ReleaseModel();
	ReleaseTexture();
	ReleaseShader();
	ReleaseCamera();
	
	if(m_showlog) printf("Resources released\n");
}

// *** Initialisation ***
IplImage* Resources::InitCapture(const char* file){
	m_capture = cvCaptureFromFile(file);
	if(!m_capture) {
		printf( "[Resources::InitCapture] Error could not read %s\n", file );
		return 0;
	}
	
	float w = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_WIDTH );
	float h = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_HEIGHT );
	
	if(m_showlog) printf("Camera settings: %.1f x %.1f\n", w, h);
	
	m_image = cvQueryFrame(m_capture);
	cvConvertImage(m_image, m_image, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
	//cvFlip(m_image, m_image, 1);
	return m_image;
}

IplImage* Resources::InitCapture(float width, float height, int camID){
	m_capture = cvCreateCameraCapture(camID);
	if(!m_capture) {
		printf( "[Resources::InitCapture] Error could not initialise camera\n" );
		return 0;
	}
	
	cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_WIDTH, width );
	cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_HEIGHT, height );
	
	float w = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_WIDTH );
	float h = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_HEIGHT );
	
	if(m_showlog) printf("Camera settings: %.1f x %.1f\n", w, h);
	
	m_image = cvQueryFrame(m_capture);
	cvConvertImage(m_image, m_image, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
	//cvFlip(m_image, m_image, 1);
	return m_image;
}

SDL_Surface* Resources::InitScreen(int width, int height, const char* name){
	if(!m_screen){
		if((SDL_Init(SDL_INIT_VIDEO)==-1)) { 
			printf("[Resources::GetScreen] Error could not initialize SDL: %s\n", SDL_GetError());
			return 0;
		}
		
		// Set mode for video output (width, height, resolution)
		m_screen = SDL_SetVideoMode( width, height, 0, SDL_OPENGL );
		if ( !m_screen) {
			printf( "[Resources::GetScreen] Error setting video mode: %s\n", SDL_GetError( ) );
			SDL_Quit();
			return 0;
		}
		SDL_WM_SetCaption(name, name);
	}else{
		printf( "[Resources::GetScreen] Warning SDL allready initialised\n" );
	}
	return m_screen;
}

ImageProcessor* Resources::InitImageProcessor(int width, int height){
	if(width==0||height==0){
		printf("[Resources::GetImageProcessor] Error ImageProcessor needs width and height for initialisation\n");
		return 0;
	}
	
	if(!m_ip)
		m_ip = new(ImageProcessor);
		
	if( !(m_ip->init(width, height)) ){
		printf("[Resources::GetImageProcessor] Error could not initialise ImageProcessor\n");
		delete(m_ip);
		m_ip = 0;
		return 0;
	}

	return m_ip;
}

Frustum* Resources::InitFrustum(){
	if(!m_frustum)
		m_frustum = new Frustum();
	
	return m_frustum;
}

// *** Release-functions ***
void Resources::ReleaseCapture(){
	if(m_capture)
		cvReleaseCapture(&m_capture);
	m_capture = 0;
}

void Resources::ReleaseScreen(){
	SDL_Quit();
	m_screen = 0;
}

void Resources::ReleaseImageProcessor(){
	if(m_ip)
		delete(m_ip);
	m_ip = 0;
}

void Resources::ReleaseFrustum(){
	if(m_frustum)
		delete(m_frustum);
	m_frustum = 0;
}

// *** Get-functions ***
IplImage* Resources::GetNewImage(){
	
	if(!m_capture){
		printf("[Resources::GetNewImage] Error camera not initialised\n" );
		return 0;
	}
	IplImage* img;
	
	try{
		img = cvQueryFrame(m_capture);
	}
	catch(char const* e){
		printf("[Resources::GetNewImage()] Warning: %s", e);
	}
	
	if(img != NULL){
		m_image = img;
		cvConvertImage(m_image, m_image, CV_CVTIMG_SWAP_RB);
	}
	
	//cvFlip(m_image, m_image, 1);
	return m_image;
}

IplImage* Resources::GetImage(){
	if(!m_image){
		return GetNewImage();
	}
	return m_image;
}

SDL_Surface* Resources::GetScreen(){
	if(!m_screen){
		printf("[Resources::GetScreen] Error SDL not initialised\n" );
		return 0;
    }
    return m_screen;	
}

ImageProcessor* Resources::GetImageProcessor(){
	if(!m_ip){
		printf("[Resources::GetImageProcessor] Error ImageProcessor not initialised\n" );
		return 0;
    }
	return m_ip;	
}

Frustum* Resources::GetFrustum(){
	if(!m_frustum){
		printf("[Resources::GetFrustum] Warning Frustum not initialised\n" );
		InitFrustum();
	}
	return m_frustum;			
}


// *** Add-functions ***
int Resources::AddParticles(int num, Particle p, const char* parname){
	int parID = SearchParticlesName(parname);
	if(parID != -1)
		return parID;
	
	Particles* ps = new Particles(num, p);
	
	char* name = new char[FN_LEN];
	strcpy(name, parname);
	
	m_particlesNameList.push_back(name);
	m_particlesList.push_back(ps);
	
	parID = m_particlesList.size() -1;
	
	if(m_showlog) printf("Particles %i loaded: %s\n", parID, name);
	
	return parID;	
}

int Resources::AddModel(TrackerModel* model, const char* name){
	// check if texture allready loaded before by comparing filename
	int modelID = -1;
	/*
	int modelID = SearchModelName(name);
	if(modelID != -1){
		printf("[Resources::AddModel] Warning model allready exists: '%s'", name);
		return modelID;	// return existing texture ID
	}
	*/
	char* tmp_name = new char[FN_LEN];
	strcpy(tmp_name, name);
	
	// put model into texture list
	m_modelNameList.push_back(tmp_name);
	m_modelList.push_back(model);
	
	modelID = m_modelList.size()-1;
	
	if(m_showlog) printf("TrackerModel %i loaded: %s\n", modelID, name);
	
	return modelID;
}


int	Resources::AddPlyModel(const char* filename){
	bool loaded = false;
	
	// check if texture allready loaded before by comparing filename
	int modelID = SearchModelName(filename);
	if(modelID != -1)
		return modelID;	// return existing texture ID
	
	// texture doesn't exist and needs to be loaded
	char fullname[FN_LEN];
	sprintf(fullname, "%s%s", m_modelPath, filename);
	TrackerModel* model = new TrackerModel();
	
	if(!m_modelloader.LoadPly(*model, fullname)){
		printf("[Resources::AddModel] Error failed to load model %s\n", fullname);
		delete(model);
		return -1;
	}
	model->computeEdges();
	model->Update();
	
	
	char* name = new char[FN_LEN];
	strcpy(name, filename);
	
	// put model into texture list
	m_modelNameList.push_back(name);
	m_modelList.push_back(model);
	
	modelID = m_modelList.size()-1;
	
	if(m_showlog) printf("TrackerModel %i loaded: %s\n", modelID, name);
	
	return modelID;
}


int	Resources::AddTexture(const char* filename, const char* texturename){
	bool loaded = false;
	int texID;
	
	if(!filename && !texturename){
		printf("[Resources::AddTexture] Error no filename nor texturename given\n");
		return -1;
	}
	
	// check if texture allready loaded before by comparing filename
	if(filename)
		texID = SearchTextureName(filename);
	else
		texID = SearchTextureName(texturename);

	if(texID != -1)
		return texID;	// return existing texture ID
	
	// texture doesn't exist and needs to be loaded
	char fullname[FN_LEN];
	sprintf(fullname, "%s%s", m_texturePath, filename);
	Texture* texture = new Texture();
		
	if(filename)
		loaded = texture->load(fullname);
	else
		loaded = true;
	
	if(!loaded){
		printf("[Resources::AddTexture] Error failed to load texture %s\n", fullname);
		delete(texture);
		return -1;
	}
	
	char* name = new char[FN_LEN];
	if(filename)
		strcpy(name, filename);
	else if(texturename)
		strcpy(name, texturename);
	
	// put model into texture list
	m_textureNameList.push_back(name);
	m_textureList.push_back(texture);
	
	texID = m_textureList.size()-1;
	
	if(m_showlog) printf("Texture %i loaded: %s\n", texID, name);
	
	return texID;	
}

int	Resources::AddShader(	const char* shadername,
							const char* vertex_file,
							const char* fragment_file,
							const char* header)
{
	
	int shaderID=-1;
	/*
	// check if texture allready loaded before by comparing filename
	int shaderID = SearchShaderName(shadername);
	if(shaderID != -1)
		return shaderID;	// return existing texture ID
	*/
	
	// texture doesn't exist and needs to be loaded
	char vertex_fullname[FN_LEN];
	char fragment_fullname[FN_LEN];
	char header_fullname[FN_LEN];
	sprintf(vertex_fullname, "%s%s", m_shaderPath, vertex_file);
	sprintf(fragment_fullname, "%s%s", m_shaderPath, fragment_file);
	sprintf(header_fullname, "%s%s", m_shaderPath, header);
		
	if(vertex_file)
		vertex_file = &vertex_fullname[0];
	if(fragment_file)
		fragment_file = &fragment_fullname[0];
	if(header)
		header = &header_fullname[0];
	
	Shader* shader = new Shader(vertex_file, fragment_file, header);
	
	if(!shader->getStatus()){
		printf("[Resources::AddShader] Error failed to load shader %s\n", shadername);
		printf("[Resources::AddShader]   Vertex shader: '%s'\n", vertex_fullname);
		printf("[Resources::AddShader]   Fragment shader: '%s'\n", fragment_fullname);
		printf("[Resources::AddShader]   Header shader: '%s'\n", header_fullname);
		delete(shader);
		return -1;
	}
	
	char* name = new char[FN_LEN];
	strcpy(name, shadername);
	
	// put model into texture list
	m_shaderNameList.push_back(name);
	m_shaderList.push_back(shader);
	
	shaderID = m_shaderList.size()-1;
	
	if(m_showlog) printf("Shader %i loaded: %s\n", shaderID, name);
	
	return shaderID;
}

int	Resources::AddCamera(const char* camname){
	int camID = SearchCameraName(camname);
	if(camID != -1)
		return camID;
	
	Camera* camera = new Camera();
	
	char* name = new char[FN_LEN];
	strcpy(name, camname);
	
	m_cameraNameList.push_back(name);
	m_cameraList.push_back(camera);
	
	camID = m_cameraList.size()-1;
	
	if(m_showlog) printf("Camera %i loaded: %s\n", camID, name);
	
	return camID;	
}



// *** Release
void Resources::ReleaseParticles(){
	// release Particles
	ParticlesList::iterator it_particles = m_particlesList.begin();
	while(it_particles != m_particlesList.end()){
		delete(*it_particles);
		it_particles++;
	}
	// release Particlesnames
	NameList::iterator it_name = m_particlesNameList.begin();
	while(it_name != m_particlesNameList.end()){
		delete(*it_name);
		it_name++;
	}
}

void Resources::ReleaseModel(){
	// release Models
	ModelList::iterator it_model = m_modelList.begin();
	while(it_model != m_modelList.end()){
		delete(*it_model);
		it_model++;
	}
	// release Modelnames
	NameList::iterator it_name = m_modelNameList.begin();
	while(it_name != m_modelNameList.end()){
		delete(*it_name);
		it_name++;
	}	
}

void Resources::ReleaseTexture(){
	// release Textures
	TextureList::iterator it_texture = m_textureList.begin();
	while(it_texture != m_textureList.end()){
		delete(*it_texture);
		it_texture++;
	}
 	// release Texturenames
	NameList::iterator it_name = m_textureNameList.begin();
	while(it_name != m_textureNameList.end()){
		delete(*it_name);
		it_name++;
	}
}

void Resources::ReleaseShader(){
	// release Shader
	ShaderList::iterator it_shader = m_shaderList.begin();
	while(it_shader != m_shaderList.end()){
		delete(*it_shader);
		it_shader++;
	}
	// release Shadernames
	NameList::iterator it_name = m_shaderNameList.begin();
	while(it_name != m_shaderNameList.end()){
		delete(*it_name);
		it_name++;
	}
}

void Resources::ReleaseCamera(){
	// release Camera
	CameraList::iterator it_camera = m_cameraList.begin();
	while(it_camera != m_cameraList.end()){
		delete(*it_camera);
		it_camera++;
	}
	// release Cameranames
	NameList::iterator it_name = m_cameraNameList.begin();
	while(it_name != m_cameraNameList.end()){
		delete(*it_name);
		it_name++;
	}
}

// *** Search-functions ***
int Resources::SearchParticlesName(const char* name){
	return SearchName(&m_particlesNameList, name);
}

int	Resources::SearchModelName(const char* filename){
	return SearchName(&m_modelNameList, filename);
}

int	Resources::SearchTextureName(const char* filename){
	return SearchName(&m_textureNameList, filename);
}

int	Resources::SearchShaderName(const char* filename){
	return SearchName(&m_shaderNameList, filename);
}

int	Resources::SearchCameraName(const char* name){
	return SearchName(&m_cameraNameList, name);
}
