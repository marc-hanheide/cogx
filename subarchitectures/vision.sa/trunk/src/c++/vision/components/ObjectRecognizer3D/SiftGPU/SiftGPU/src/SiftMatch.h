////////////////////////////////////////////////////////////////////////////
//	File:		SiftMatch.h
//	Author:		Changchang Wu
//	Description :	interface for the SiftMatchCG
////
//	Copyright (c) 2007 University of North Carolina at Chapel Hill
//	All Rights Reserved
//
//	Permission to use, copy, modify and distribute this software and its
//	documentation for educational, research and non-profit purposes, without
//	fee, and without a written agreement is hereby granted, provided that the
//	above copyright notice and the following paragraph appear in all copies.
//	
//	The University of North Carolina at Chapel Hill make no representations
//	about the suitability of this software for any purpose. It is provided
//	'as is' without express or implied warranty. 
//
//	Please send BUG REPORTS to ccwu@cs.unc.edu
//
////////////////////////////////////////////////////////////////////////////




#if !defined(GPU_SIFT_MATCH_H)
#define GPU_SIFT_MATCH_H
class GLTexImage;
class ProgramCG;
class SiftMatchCG
{
private:
	//tex storage
	GLTexImage _texLoc[2];
	GLTexImage _texDes[2];
	GLTexImage _texDot;
	GLTexImage _texMatch[2];

	//programs
	ProgramCG * s_multiply;
	ProgramCG * s_guided_mult;
	ProgramCG * s_col_max;
	ProgramCG * s_row_max;
	CGparameter _param_multiply_tex1;
	CGparameter _param_multiply_tex2;
	CGparameter _param_multiply_size;
	CGparameter _param_rowmax_param;
	CGparameter _param_colmax_param;

	///guided matching
	CGparameter _param_guided_mult_tex1;
	CGparameter _param_guided_mult_tex2;
	CGparameter _param_guided_mult_texl1;
	CGparameter _param_guided_mult_texl2;
	CGparameter _param_guided_mult_h;
	CGparameter _param_guided_mult_f;
	CGparameter _param_guided_mult_param;
	//
	int _max_sift; 
	int _num_sift[2];
	int _id_sift[2];
	int _have_loc[2];

	//gpu parameter

	int _use_packed_des;
	int _sift_per_stripe;
	int _sift_num_stripe;
	int	_sift_per_row;
	int	_pixel_per_sift;
	int	_sift_per_match_row;
	//
	int _initialized;
	//
	vector<float> sift_buffer; 
private:
	void AllocateSiftMatch();
	void LoadSiftMatchShaders();
	int  GetBestMatch(int max_match, int match_buffer[][2], float distmax, float ratiomax, int mbm);
public:
	SiftMatchCG(int max_sift = 4096);
	virtual ~SiftMatchCG();
	void InitSiftMatchCG();
	void SetMaxSift(int max_sift);
	void SetDescriptors(int index, int num, const unsigned char * descriptor, int id = -1);
	void SetDescriptors(int index, int num, const float * descriptor, int id = -1);
	void SetFeautreLocation(int index, const float* locatoins, int gap);
	int  GetSiftMatch(int max_match, int match_buffer[][2], float distmax, float ratiomax, int mbm);
	int  GetGuidedSiftMatch(int max_match, int match_buffer[][2], float distmax, float ratiomax,
		float H[3][3], float hdistmax, float F[3][3], float fdistmax, int mbm);
};


#endif

