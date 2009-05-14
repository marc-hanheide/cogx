#ifndef __uol_matlab_engine_hh__
#define __uol_matlab_engine_hh__

/******************************************************************************
 * $Author::                                                                 $*
 * $Date::                                                                   $*
 * $Revision::                                                               $*
 *****************************************************************************/

#include "engine.h"


class uol_matlab_engine
{
	protected:
		/// A reference to Matlab engine.
		Engine* matlab_engine_;

		char* output_buffer_;

    char last_command_[256];

	public:
		/// Default constructor that opens Matlab engine.
		uol_matlab_engine(unsigned _buffer_length = 0);

		/// Destructor that closes the engine and frees resources.
		~uol_matlab_engine();

		/**
		 * Puts the variable into MATLAB engine workspace.
		 *
		 * @return 0 is successfull and 1 if an error occurs
		 */
		int put_variable(const char* _name, const mxArray* _mp);

		int put_variable(const char* _name, const double* _data_ptr,
				unsigned _n_rows, unsigned _n_cols);

		int put_variable1(const char* _name, const double* _data_ptr,
				int _n_dimensions, int* _dimensions);

		/**
		 * Copy value from MATLAB engine workspace.
		 */
		mxArray* get_variable(const char* _name);

		/**
		 * Gets the scalar value for the given variable.
		 */
		double get_scalar(const char* _name);

		/**
		 * Gets the specified matrix. The dimension of the matrix is returned
		 * in _n_rows and _n_cols.
		 */
		double* get_matrix(const char* _name, int& _n_rows, int& _n_cols);

		/**
		 * Evaluates the expression in string.
		 *
		 * @return 0 on success, nonzero if the matlab session already closed
		 */
		int eval(const char* _string);

		/**
		 * Sets the visibility of Matlab engine.
		 */
		void set_visible(bool _visible);

    void log_matlab();

		char* get_buffer() { return this->output_buffer_ + 2; }
}; // class uol_matlab_engine

#endif

