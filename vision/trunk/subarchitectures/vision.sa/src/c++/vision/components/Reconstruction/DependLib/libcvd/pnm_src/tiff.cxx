/*                       
	This file is part of the CVD Library.

	Copyright (C) 2005 The Authors

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 
    51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "cvd/internal/io/tiff.h"
#include "cvd/image_io.h"
#include "cvd/config.h"
#include <tiffio.h>
#include <algorithm>
#include <vector>
#include <iostream>

using namespace CVD;
using namespace CVD::TIFF;
using namespace CVD::Exceptions::Image_IO;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
//
// Private implementation of TIFF reading
//

class CVD::TIFF::TIFFPimpl
{
	public:
		TIFFPimpl(istream&);
		~TIFFPimpl();
		ImageRef size();
		string datatype();
		template<class C> void get_raw_pixel_line(C* data);
		void get_raw_pixel_line(bool* data);

	private:
		istream& i;
		unsigned long row;
		ImageRef my_size;
		string   type;
		streamoff length;
		::TIFF* tif;
		bool use_cooked_rgba_interface;
		bool inverted_grey;

		vector<uint32> raster_data;
		vector<uint8>  bool_rowbuf;

		static tsize_t write(thandle_t vis, tdata_t data, tsize_t count);
		static tsize_t read(thandle_t vis, tdata_t data, tsize_t count);
		static toff_t seek(thandle_t vis, toff_t off, int dir);
		static toff_t size(thandle_t vis);
		static int close(thandle_t vis);
		static int map(thandle_t, tdata_t*, toff_t*);
		static void unmap(thandle_t, tdata_t, toff_t);
};


tsize_t TIFFPimpl::read(thandle_t vis, tdata_t data, tsize_t count)
{
	TIFFPimpl* i = (TIFFPimpl*)vis;
	i->i.read((char*)data, count);
	return i->i.gcount();
}

//tsize_t tiff_in::write(thandle_t vis, tdata_t data, tsize_t count)
tsize_t TIFFPimpl::write(thandle_t, tdata_t, tsize_t)
{
	return 0;
}

toff_t TIFFPimpl::seek(thandle_t vis, toff_t off, int dir)
{
	TIFFPimpl* i = (TIFFPimpl*)vis;

	if(dir == SEEK_SET)
		i->i.seekg(off, ios_base::beg);
	else if(dir == SEEK_CUR)
		i->i.seekg(off, ios_base::cur);
	else if(dir == SEEK_END)
		i->i.seekg(off, ios_base::end);

	return i->i.tellg();
}

toff_t TIFFPimpl::size(thandle_t vis)
{
	TIFFPimpl* ii = (TIFFPimpl*)vis;
	return ii->length;
}

int TIFFPimpl::close(thandle_t)
{
	return 0;
}

int TIFFPimpl::map(thandle_t, tdata_t*, toff_t*)
{
	return 0;
}

void TIFFPimpl::unmap(thandle_t, tdata_t, toff_t)
{
}


static const int error_size=512;
static char error_msg[error_size]="";

static void tiff_error_handler(const char*, const char* fmt, va_list ap)
{
	int n = vsnprintf(error_msg, error_size, fmt, ap);
	if(n == error_size)
		error_msg[n-1] = 0;
}


//Compile-error free geryscale inverter.
//Does nothing for unknown types
template<class C> void invert(C* data, long num)
{
	for(long n=0; n< num; n++)
		data[n] =  Pixel::traits<C>::max_intensity - data[n];
}

void attempt_invert(...) {}
void attempt_invert(bool* data, long num) { invert(data, num);}
void attempt_invert(unsigned char* data, long num) { invert(data, num);}
void attempt_invert(unsigned short* data, long num) { invert(data, num);}
void attempt_invert(float* data, long num) { invert(data, num);}
void attempt_invert(double* data, long num) { invert(data, num);}



template<class T> void TIFFPimpl::get_raw_pixel_line(T* d)
{
	if(datatype() != PNM::type_name<T>::name())
		throw ReadTypeMismatch(datatype(), PNM::type_name<T>::name());

	if(row  > (unsigned long)my_size.y)
		throw InternalLibraryError("CVD", "Read past end of image.");
	
	if(use_cooked_rgba_interface)
	{
		uint32* raster = &raster_data[row*my_size.x];
		uint32* end = raster + my_size.x;

		//We will only ever get here if the type is Rgba
		Rgba<unsigned char>* data = reinterpret_cast<Rgba<unsigned char>* >(d);
	
		for(;raster < end; raster++, data++)
		{
			data->red   = TIFFGetR(*raster);
			data->green = TIFFGetG(*raster);
			data->blue  = TIFFGetB(*raster);
			data->alpha = TIFFGetA(*raster);
		}

		row ++;
	}
	else
	{	
		if(TIFFReadScanline(tif, (void*)d, row) == -1)
			throw MalformedImage(error_msg);

		if(inverted_grey)
			attempt_invert(d, my_size.x);

		row++;
	}
}

void TIFFPimpl::get_raw_pixel_line(bool* d)
{
	if(datatype() != PNM::type_name<bool>::name())
		throw ReadTypeMismatch(datatype(), PNM::type_name<bool>::name());

	if(TIFFReadScanline(tif, (void*)&bool_rowbuf[0], row) == -1)
		throw MalformedImage(error_msg);

	//Unpack the bools
	for(int i=0; i < my_size.x  ;i++)
		d[i] = (bool_rowbuf[i/8] >> (7-i%8)) & 1;

	if(inverted_grey)
		invert(d, my_size.x);

	row++;
}

string TIFFPimpl::datatype()
{
	return type;
}

ImageRef TIFFPimpl::size()
{
	return my_size;
}

TIFFPimpl::~TIFFPimpl()
{	
	TIFFClose(tif);
}


TIFFPimpl::TIFFPimpl(istream& is)
:i(is),row(0)
{
	TIFFSetErrorHandler(tiff_error_handler);
	
	//Find out the file size, and the suitability of the stream
	i.seekg(0, ios_base::end);
	length = i.tellg();
	if(length == -1)
		throw UnseekableIstream("TIFF");
	i.seekg(0, ios_base::beg);


	tif = TIFFClientOpen("std::istream", "r", this, 
						 read, write, seek, close, size, map, unmap);


	if(tif == NULL)
		throw MalformedImage(error_msg);

	//Libtiff types
	uint32 w=0, h=0;
	uint16 bitspersample=0, spp=0, sampleformat=0, photo=0, pl_type=0;


	TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
	TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);	
	TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bitspersample);
	TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &spp);
	TIFFGetField(tif, TIFFTAG_PHOTOMETRIC, &photo);	
	TIFFGetField(tif, TIFFTAG_PLANARCONFIG, &pl_type);

	//Read the sample format. If it is missing, then it
	//defaults to unsigned int as per the spec.
	if(TIFFGetField(tif, TIFFTAG_SAMPLEFORMAT, &sampleformat) == 0)
		sampleformat = SAMPLEFORMAT_UINT;


	my_size.x = w;
	my_size.y = h;
	use_cooked_rgba_interface=1; //This is the default
	inverted_grey=0;

	//Can we use our own interface?
	//The alternative is the cooked RGBA interface which assumes all
	//the world is 8 bits RGBA. This will load almost all images, but 
	//special features, such as higher bit depths will be lost.
	//Also, the entire image has to be loaded at once, so this can use large
	//amounts of memory.
	if((photo == PHOTOMETRIC_RGB  || photo == PHOTOMETRIC_MINISWHITE || photo == PHOTOMETRIC_MINISBLACK))
	{	
		//We stand a chane here...
		if(photo == PHOTOMETRIC_MINISWHITE)
			inverted_grey=1;

		//Figure out the basic datatype
		if(sampleformat == SAMPLEFORMAT_UINT)
		{
			if(bitspersample == 1)	
				type = PNM::type_name<bool>::name();
			else if(bitspersample == 8)
				type = PNM::type_name<unsigned char>::name();
			else if(bitspersample == 16)
				type = PNM::type_name<unsigned short>::name();
			else 
				goto keep_cooked;
		}
		else if(sampleformat == SAMPLEFORMAT_IEEEFP)
		{
			if(bitspersample == 32)
				type = PNM::type_name<float>::name();
			else if(bitspersample == 64)
				type = PNM::type_name<double>::name();
			else 
				goto keep_cooked;
		}
		else
			goto keep_cooked;
		
		//Figure out the colourspace
		if(spp == 1)
			type = type;
		else if(spp == 2)
			type = "CVD::GreyAlpha<" + type  + ">";
		else if(spp == 3)
			type = "CVD::Rgb<" + type  + ">";
		else if(spp == 4)
			type = "CVD::Rgba<" + type  + ">";
		else 
			goto keep_cooked;

		use_cooked_rgba_interface=0;
	}
		
	keep_cooked:;
	if(use_cooked_rgba_interface == 1)
	{	
		//The format is "complex" and we don't know how to read it.
		type = "CVD::Rgb<unsigned char>";
		inverted_grey=0;
	}

	if(type == "bool")
		bool_rowbuf.resize((size().x + 7)/8);



	if(use_cooked_rgba_interface)
	{
		raster_data.resize(my_size.x, my_size.y);

		#ifdef CVD_INTERNAL_HAVE_TIFF_ORIENTED
			//Read the whole image
			if(TIFFReadRGBAImageOriented(tif, my_size.x, my_size.y, &raster_data[0], 0, ORIENTATION_TOPLEFT) == -1)
				throw MalformedImage(error_msg);
		#else
			//Read the whole (upside-down) image
			if(TIFFReadRGBAImage(tif, my_size.x, my_size.y, &raster_data[0], 0) == -1)
				throw MalformedImage(error_msg);

			int xs = my_size.x, ys = my_size.y;
			
			//Flip the image, a row pair at a time
			vector<uint32> buffer(xs);
			for(int top=0, bot = ys-1; top < ys/2 && top != bot; top++,bot--)
			{
				uint32* bp, *tp;

				tp = &raster_data[0] + top * xs;
				bp = &raster_data[0] + bot * xs;

				memcpy(&buffer[0], tp, xs*sizeof(uint32));
				memcpy(tp, bp,  xs*sizeof(uint32));
				memcpy(bp, &buffer[0], xs*sizeof(uint32));
			}
		#endif
	}
}




////////////////////////////////////////////////////////////////////////////////
//
// Implementation of public parts of TIFF reading
//

tiff_reader::tiff_reader(istream& i)
:t(new TIFFPimpl(i))
{}

tiff_reader::~tiff_reader()
{
}

string tiff_reader::datatype()
{
	return t->datatype();
}

string tiff_reader::name()
{
	return "TIFF";
}

ImageRef tiff_reader::size()
{
	return t->size();
};

//Mechanically generate the pixel reading calls.
#define GEN1(X) void tiff_reader::get_raw_pixel_line(X*d){t->get_raw_pixel_line(d);}
#define GEN3(X) GEN1(X) GEN1(Rgb<X>) GEN1(Rgba<X>)

GEN1(bool)
GEN3(unsigned char)
GEN3(unsigned short)
GEN3(float)
GEN3(double)
