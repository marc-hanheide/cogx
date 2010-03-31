 /**
 * @file tgLabel.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Quad patch with texture created by copying text from frame buffer.
 * @namespace TomGine
 */
 
#ifndef _TG_LABEL_H_
#define _TG_LABEL_H_
 
#include <FTGL/ftgl.h>
#include <vector>
#include <string>

#include "tgFont.h"
#include "tgTexture.h"
#include "tgPose.h"

namespace TomGine{

class tgLabel
{
private:
	float m_width;
	float m_height;
	int m_txtSize;
	int m_maxStrLen;
	
	
	tgFont* m_font;
	tgTexture* m_texture;
	std::vector<std::string> m_text;
	std::string m_fontfilename;

public:
	tgPose m_pose;
	
	/** @brief Constructor of tgLabel
	*	@param w width of label in pixels
	* @param h height of label in pixels
	* @param ttf_filename file name of true type font (ttf) to use for text */
	tgLabel();
	tgLabel(const char* ttf_filename);
	~tgLabel();
	
	void SetFont(const char* ttf_filename){ m_fontfilename = std::string(ttf_filename); }
	void AddText(const char* text);
	void CreateLabel();
	void Clear();
	void Draw();
	
};

}

#endif
