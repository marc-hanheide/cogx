
#include "Resources.h"

#include <v4r/TomGine/tgError.h>

using namespace Tracking;

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
	m_ip = 0;
	m_showlog = false;
}

Resources::~Resources(){
	ReleaseCapture();
	//ReleaseScreen();
	ReleaseImageProcessor();
	
	ReleaseShader();
	
	if(m_showlog) printf("Resources released\n");
}


// *** Initialisation ***
IplImage* Resources::InitCapture(const char* file){
	m_capture = cvCaptureFromFile(file);
	if(!m_capture) {
		char errmsg[128];
		sprintf(errmsg, "[Resources::InitCapture] Error could not read '%s'\n", file );
		throw std::runtime_error(errmsg);
	}
	
	double w = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_WIDTH );
	double h = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_HEIGHT );
	
	if(m_showlog) printf("Camera settings: %.1f x %.1f\n", w, h);
	
	m_image = cvQueryFrame(m_capture);
	cvConvertImage(m_image, m_image, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
	//cvFlip(m_image, m_image, 1);
	return m_image;
}

IplImage* Resources::InitCapture(float width, float height, int camID){
	m_capture = cvCreateCameraCapture(camID);
	if(!m_capture) {
		char errmsg[128];
		sprintf(errmsg, "[Resources::InitCapture] Error could not initialise camera\n" );
		throw std::runtime_error(errmsg);
	}
	
	cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_WIDTH, width );
	cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_HEIGHT, height );
	
	double w = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_WIDTH );
	double h = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_HEIGHT );
	
	if(m_showlog) printf("Camera settings: %.1f x %.1f\n", w, h);
	
	m_image = cvQueryFrame(m_capture);
	cvConvertImage(m_image, m_image, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
	//cvFlip(m_image, m_image, 1);
	return m_image;
}

TomGine::tgImageProcessor* Resources::InitImageProcessor(int width, int height)
{
	if(width==0||height==0){
		printf("[Resources::GetImageProcessor] Error TomGine::tgImageProcessor needs width and height for initialisation\n");
		return 0;
	}
	
	if(!m_ip)
		m_ip = new TomGine::tgImageProcessor( width, height );
		
	return m_ip;
}

// *** Release-functions ***
void Resources::ReleaseCapture(){
	if(m_capture)
		cvReleaseCapture(&m_capture);
	m_capture = 0;
}

void Resources::ReleaseImageProcessor(){
	if(m_ip)
		delete(m_ip);
	m_ip = 0;
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
// 		cvConvertImage(m_image, m_image, CV_CVTIMG_SWAP_RB);
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

TomGine::tgImageProcessor* Resources::GetImageProcessor(){
	if(!m_ip){
		printf("[Resources::GetImageProcessor] Error TomGine::tgImageProcessor not initialised\n" );
		return 0;
    }
	return m_ip;	
}

TomGine::tgShader* Resources::GetShader(int id){
	return m_shaderList[id];
}

// *** Add
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
	sprintf(vertex_fullname, "%s%s", m_shaderPath.c_str(), vertex_file);
	sprintf(fragment_fullname, "%s%s", m_shaderPath.c_str(), fragment_file);
	sprintf(header_fullname, "%s%s", m_shaderPath.c_str(), header);

//	printf("[Resources::AddShader] %s\n", fragment_fullname);
		
	if(vertex_file)
		vertex_file = &vertex_fullname[0];
	if(fragment_file)
		fragment_file = &fragment_fullname[0];
	if(header)
		header = &header_fullname[0];

	TomGine::tgShader* shader = new TomGine::tgShader(vertex_file, fragment_file, header);

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
	
	if(m_showlog) printf("TomGine::tgShader %i loaded: %s\n", shaderID, name);
	
	return shaderID;
}

// *** Release
void Resources::ReleaseShader(){
	// release TomGine::tgShader
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

void Resources::ReleaseShader(int id){
	if(m_shaderList[id])
		delete(m_shaderList[id]);
	m_shaderList[id] = 0;
	if(m_showlog) printf("TomGine::tgShader %i released\n", id);
#ifdef DEBUG
	TomGine::tgCheckError("[Resources::ReleaseShader]");
#endif
}


// *** Search-functions ***
int	Resources::SearchShaderName(const char* filename){
	return SearchName(&m_shaderNameList, filename);
}
