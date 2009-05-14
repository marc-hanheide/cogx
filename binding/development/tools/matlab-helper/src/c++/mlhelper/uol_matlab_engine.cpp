/******************************************************************************
 * $Author::                                                                 $*
 * $Date::                                                                   $*
 * $Revision::                                                               $*
 *****************************************************************************/

#include <string.h>
#include <iostream>
#include "uol_matlab_engine.h"

uol_matlab_engine::uol_matlab_engine(unsigned _buffer_length /* = 0 */)
{
	// Open the Matlab engine.
	std::cout << "Opening Matlab engine...\n";
	if (!(this->matlab_engine_ = engOpen("\0"))) {
		std::cout << "Cannot open the Matlab engine.\n";
	};

	if (_buffer_length > 0)
	{
		this->output_buffer_ = new char[_buffer_length + 1];
		this->output_buffer_[_buffer_length] = '\0';
		engOutputBuffer(this->matlab_engine_, this->output_buffer_, 
				_buffer_length);
	} // if
	else
	{
		this->output_buffer_ = new char[3];
		this->output_buffer_[2] = '\0';
	} // if - else
} // uol_matlab_engine::uol_matlab_engine

uol_matlab_engine::~uol_matlab_engine()
{
	// Close the Matlab engine.
	engClose(this->matlab_engine_);

	delete [] this->output_buffer_;
} // uol_matlab_engine::~uol_matlab_engine

int uol_matlab_engine::put_variable(const char* _name, const mxArray* _mp)
{
	// Put the variable into MATLAB engine.
	return engPutVariable(this->matlab_engine_, _name, _mp);
} // uol_matlab_engine::put_variable

mxArray* uol_matlab_engine::get_variable(const char* _name)
{
	return engGetVariable(this->matlab_engine_, _name);
} // uol_matlab_engine::get_variable

double uol_matlab_engine::get_scalar(const char* _name)
{
	// Get the variable.
	mxArray* mp = this->get_variable(_name);

	// Get the array pointer and return the first value. Note that this also
	// works for non-scalar matrices where only the first element is returned.
	return mxGetPr(mp)[0];
} // uol_matlab_engine::get_scalar

double* uol_matlab_engine::get_matrix(const char* _name, 
		int& _n_rows, int& _n_cols)
{
	// Get the variable.
	mxArray* mp = this->get_variable(_name);

	_n_rows = mxGetM(mp);
	_n_cols = mxGetN(mp);

	return mxGetPr(mp);
} // uol_matlab_engine::get_matrix

int uol_matlab_engine::eval(const char* _string)
{
	this->output_buffer_[2] = '\0';
	int ret = engEvalString(this->matlab_engine_, _string);
  sprintf(this->last_command_, "%s", _string);
//	std::cout << _string << std::endl;
//	std::cout << this->get_buffer();

	return ret;
} // uol_matlab_engine::eval

int uol_matlab_engine::put_variable(const char* _name, const double* _data_ptr,
		unsigned _n_rows, unsigned _n_cols)
{
	// Prepare double matrix of specified dimensions.
	mxArray* mp = mxCreateDoubleMatrix(_n_rows, _n_cols, mxREAL);
	memcpy((void*)mxGetPr(mp), (void*)_data_ptr,
			_n_rows * _n_cols * sizeof(double));

	return this->put_variable(_name, mp);
} // uol_matlab_engine::put_variable

int uol_matlab_engine::put_variable1(const char* _name, 
		const double* _data_ptr,
		int _n_dimensions, int* _dimensions)
{
	mxArray* mp;
	if (_data_ptr == NULL)
	{
		mp = mxCreateDoubleMatrix(0, 0, mxREAL);
	}
	else
	{
		mp = mxCreateNumericArray(_n_dimensions, _dimensions,
				mxDOUBLE_CLASS, mxREAL);

		int n_dim = mxGetNumberOfDimensions(mp);
		const int* dim = mxGetDimensions(mp);

		unsigned length = mxGetNumberOfElements(mp);
		memcpy((void*)mxGetPr(mp), (void*)_data_ptr, length * sizeof(double));
	} // if - else

	return this->put_variable(_name, mp);
} // uol_matlab_engine::put_variable

void uol_matlab_engine::set_visible(bool _visible)
{
	engSetVisible(this->matlab_engine_, _visible);
} // uol_matlab_engine::set_visible

void uol_matlab_engine::log_matlab()
{
  std::cout << ">> " << this->last_command_ << std::endl;
  std::cout <<  this->get_buffer() << std::endl;
} // uol_matlab_engine::log_matlab
