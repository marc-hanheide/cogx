#include <TooN/optimization/brent.h>
#include <utility>
#include <cmath>
#include <cassert>
#include <cstdlib>

namespace TooN{

	namespace Internal{
	

	///Turn a multidimensional function in to a 1D function by specifying a
	///point and direction. A nre function is defined:
	////\f[
	/// g(a) = \Vec{s} + a \Vec{d}
	///\f]
	///@ingroup gOptimize
	template<int Size, typename Precision, typename Func> struct LineSearch
	{
		const Vector<Size, Precision>& start; ///< \f$\Vec{s}\f$
		const Vector<Size, Precision>& direction;///< \f$\Vec{d}\f$

		const Func& f;///< \f$f(\cdotp)\f$

		///Set up the line search class.
		///@param s Start point, \f$\Vec{s}\f$.
		///@param d direction, \f$\Vec{d}\f$.
		///@param func Function, \f$f(\cdotp)\f$.
		LineSearch(const Vector<Size, Precision>& s, const Vector<Size, Precision>& d, const Func& func)
		:start(s),direction(d),f(func)
		{}
		
		///@param x Position to evaluate function
		///@return \f$f(\vec{s} + x\vec{d})\f$
		Precision operator()(Precision x) const
		{
			return f(start + x * direction); 
		}
	};
	
	///Bracket a 1D function by searching forward from zero. The assumption
	///is that a minima exists in \f$f(x),\ x>0\f$, and this function searches
	///for a bracket using exponentially growning or shrinking steps.
	///@param a_val The value of the function at zero.
	///@param func Function to bracket
	///@param initial_lambda Initial stepsize
	///@return <code>m[i][0]</code> contains the values of \f$x\f$ for the bracket, in increasing order, 
	///        and <code>m[i][1]</code> contains the corresponding values of \f$f(x)\f$.
	///@ingroup gOptimize
	template<typename Precision, typename Func> Matrix<3,2,Precision> bracket_minimum_forward(Precision a_val, const Func& func, Precision initial_lambda=1)
	{
		//Get a, b, c to  bracket a minimum along a line
		Precision a, b, c, b_val, c_val;

		a=0;

		//Search forward in steps of lambda
		Precision lambda=initial_lambda;
		b = lambda;
		b_val = func(b);

		if(b_val < a_val) //We've gone downhill, so keep searching until we go back up
		{
			for(;;)
			{
				lambda *= 2;
				c = lambda;
				c_val = func(c);

				if(c_val > 	b_val) // we have a bracket
					break;
				else
				{
					a = b;
					a_val = b_val;
					b=c;
					b_val=c_val;

				}
			}
		}
		else //We've overshot the minimum, so back up
		{
			c = b;
			c_val = b_val;
			//Here, c_val > a_val

			for(;;)
			{
				lambda *= .5;
				b = lambda;
				b_val = func(b);

				if(b_val < a_val)// we have a bracket
					break;
				else //Contract the bracket
				{
					c = b;
					c_val = b_val;
				}
			}
		}
		
		Matrix<3,2> ret;
		ret[0] = makeVector(a, a_val);
		ret[1] = makeVector(b, b_val);
		ret[2] = makeVector(c, c_val);

		return ret;
	}

}


/** This class provides a nonlinear conjugate-gradient optimizer. The following
code snippet will perform an optimization on the Rosenbrock Bananna function in
two dimensions:

@code
double Rosenbrock(const Vector<2>& v)
{
		return sq(1 - v[0]) + 100 * sq(v[1] - sq(v[0]));
}

Vector<2> RosenbrockDerivatives(const Vector<2>& v)
{
	double x = v[0];
	double y = v[1];

	Vector<2> ret;
	ret[0] = -2+2*x-400*(y-sq(x))*x;
	ret[1] = 200*y-200*sq(x);

	return ret;
}

int main()
{
	ConjugateGradient<2> cg(makeVector(0,0), Rosenbrock, RosenbrockDerivatives);

	while(!cg.iterate(Rosenbrock, RosenbrockDerivatives))
		cout << "y_" << iteration << " = " << cg.y << endl;

	cout << "Optimal value: " << cg.y << endl;
}
@endcode

The chances are that you will want to read the documentation for
ConjugateGradient::ConjugateGradient and ConjugateGradient::iterate.

Linesearch is currently performed using golden-section search and conjugate
vector updates are performed using the Polak-Ribiere equations.  There many
tunable parameters, and the internals are readily accessible, so alternative
termination conditions etc can easily be substituted. However, ususally these
will not be necessary.

@ingroup gOptimize
*/
template<int Size, class Precision=double> struct ConjugateGradient
{
	const int size;      ///< Dimensionality of the space. 
	Vector<Size> g;      ///< Gradient vector used by the next call to iterate()
	Vector<Size> h;      ///< Conjugate vector to be searched along in the next call to iterate()
	Vector<Size> old_g;  ///< Gradient vector used to compute $h$ in the last call to iterate()
	Vector<Size> old_h;  ///< Conjugate vector searched along in the last call to iterate()
	Vector<Size> x;      ///< Current position (best known point)
	Vector<Size> old_x;  ///< Previous best known point (not set at construction)
	Precision y;         ///< Function at \f$x\f$
	Precision old_y;     ///< Function at  old_x

	Precision tolerance; ///< Tolerance used to determine if the optimization is complete. Defaults to square root of machine precision.
	Precision epsilon;   ///< Additive term in tolerance to prevent excessive iterations if \f$x_\mathrm{optimal} = 0\f$. Known as \c ZEPS in numerical recipies. Defaults to 1e-20
	int       max_iterations; ///< Maximum number of iterations. Defaults to \c size\f$*100\f$

	Precision bracket_initial_lambda;///< Initial stepsize used in bracketing the minimum for the line search. Defaults to 1.
	Precision linesearch_tolerance; ///< Tolerance used to determine if the linesearch is complete. Defaults to square root of machine precision.
	Precision linesearch_epsilon; ///< Additive term in tolerance to prevent excessive iterations if \f$x_\mathrm{optimal} = 0\f$. Known as \c ZEPS in numerical recipies. Defaults to 1e-20
	int linesearch_max_iterations;  ///< Maximum number of iterations in the linesearch. Defaults to 100.

	int iterations; ///< Number of iterations performed
	
	///Initialize the ConjugateGradient class with sensible values.
	///@param start Starting point, \e x
	///@param func  Function \e f  to compute \f$f(x)\f$
	///@param deriv  Function to compute \f$\nabla f(x)\f$
	template<class Func, class Deriv> ConjugateGradient(const Vector<Size>& start, const Func& func, const Deriv& deriv)
	: size(start.size()),
	  g(size),h(size),old_g(size),old_h(size),x(start),old_x(size)
	{
		using std::numeric_limits;

		x = start;
		
		//Start with the conjugate direction aligned with
		//the gradient
		g = deriv(x);
		h = g;

		y = func(x);
		old_y = y;

		tolerance = sqrt(numeric_limits<Precision>::epsilon());
		epsilon = 1e-20;
		max_iterations = size * 100;

		bracket_initial_lambda = 1;

		linesearch_tolerance =  sqrt(numeric_limits<Precision>::epsilon());
		linesearch_epsilon = 1e-20;
		linesearch_max_iterations=100;

		iterations=0;
	}
	

	///Perform a linesearch from the current point (x) along the current
	///conjugate vector (h).  The linesearch does not make use of derivatives.
	///You probably do not want to use this function. See iterate() instead.
	///This function updates:
	/// - x
	/// - old_c
	/// - y
	/// - old_y
	/// - iterations
	/// Note that the conjugate direction and gradient are not updated.
	/// @param func Functor returning the function value at a given point. 
	template<class Func> void find_next_point(const Func& func)
	{
		Internal::LineSearch<Size, Precision, Func> line(x, -h, func);

		//Always search in the conjugate direction (h)

		//First bracket a minimum. 
		Matrix<3,2,Precision> bracket = Internal::bracket_minimum_forward(y, line, bracket_initial_lambda);

		double a = bracket[0][0];
		double b = bracket[1][0];
		double c = bracket[2][0];

		double a_val = bracket[0][1];
		double b_val = bracket[1][1];
		double c_val = bracket[2][1];

		//We should have a bracket here
		assert(a < b && b < c);
		assert(a_val > b_val && b_val < c_val);
		
		//Find the real minimum
		Vector<2, Precision>  m = brent_line_search(a, b, c, b_val, line, linesearch_max_iterations, linesearch_tolerance, linesearch_epsilon); 
	
		assert(m[0] >= a && m[0] <= c);
		assert(m[1] <= b_val);
		
		//Update the current position and value
		old_y = y;
		old_x = x;

		x -= m[0] * h;
		y = m[1];
		
		iterations++;
	}
	
	///Check to see it iteration should stop. You probably do not want to use
	///this function. See iterate() instead. This function updates nothing.
	bool finished()
	{
		using std::abs;
		return iterations > max_iterations || 2*abs(y - old_y) <= tolerance * (abs(y) + abs(old_y) + epsilon);
	}
	
	///After an iteration, update the gradient and conjugate using the
	///Polak-Ribiere equations.  /@param deriv Functor to compute derivatives at
	///the specified point. This function updates:
	///- g
	///- old_g
	///- h
	///- old_h
	template<class Deriv> void update_vectors_PR(const Deriv& deriv)
	{
		//Update the position, gradient and conjugate directions
		old_g = g;
		old_h = h;

		g = deriv(x);
		//Precision gamma = (g * g - oldg*g)/(oldg * oldg);
		Precision gamma = (g * g - old_g*g)/(old_g * old_g);
		h = g + gamma * old_h;
	}
	
	///Use this function to iterate over the optimization. Note that after
	///iterate returns false, g, h, old_g and old_h will not have been
	///updated.
	///This function updates:
	/// - x
	/// - old_c
	/// - y
	/// - old_y
	/// - iterations
	/// - g*
	/// - old_g*
	/// - h*
	/// - old_h*
	/// *'d variables not updated on the last iteration.
	///@param func Functor returning the function value at a given point. 
	///@param deriv Functor to compute derivatives at the specified point.
	///@return Whether to continue.
	template<class Func, class Deriv> bool iterate(const Func& func, const Deriv& deriv)
	{
		find_next_point(func);

		if(!finished())
		{
			update_vectors_PR(deriv);
			return 1;
		}
		else
			return 0;
	}
};

}
