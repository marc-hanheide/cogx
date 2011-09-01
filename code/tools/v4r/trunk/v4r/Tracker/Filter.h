

#ifndef _TRACKING_SMOOTH_FILTER_
#define _TRACKING_SMOOTH_FILTER_

 
namespace Tracking{

class FloatFilter
{
public:
    FloatFilter(float a=0.0f){ this->a = a; b = 1.0f - a; z = 0.0f; }

	inline void Set(const float &z) { this->z = z; }

	operator float() const { return z; }
	FloatFilter& operator=(const float &in){
		if(!isnan(in))
			z = (in * b) + (z * a);
		return (*this);
	}

private:
    float a, b, z;
};

class FloatFilterRise
{
public:
    FloatFilterRise(float a=0.0f){ this->a = a; b = 1.0f - a; z = 0.0f; }

	inline void Set(const float &z) { this->z = z; }

	operator float() const { return z; }
	FloatFilterRise& operator=(const float &in){
		if(!isnan(in)){
			if(in>=z)
				z = (in * b) + (z * a);
			else
				z = in;
		}
		return (*this);
	}

private:
    float a, b, z;
};

class FloatFilterFall
{
public:
    FloatFilterFall(float a=0.0f){ this->a = a; b = 1.0f - a; z = 0.0f; }

	inline void Set(const float &z) { this->z = z; }

	operator float() const { return z; }
	FloatFilterFall& operator=(const float &in){
		if(!isnan(in)){
			if(in<=z)
				z = (in * b) + (z * a);
			else
				z = in;
		}
		return (*this);
	}

private:
    float a, b, z;
};


} 
 
#endif
