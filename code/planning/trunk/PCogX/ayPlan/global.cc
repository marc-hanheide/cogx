// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "global.hh"

namespace MemoryManagement{
    vector<void*> toDelete;
}

int Compression::compress(unsigned char* input, uint input_size,
                          unsigned char* output, uint output_size,
                          int level)
{
    int ret;
    uint have;
    z_stream strm;
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
            
    ret = deflateInit(&strm, level);
            
    if (ret != Z_OK)
        return ret;

    strm.avail_in = input_size;
    strm.next_in = input;

    strm.avail_out = output_size;
    strm.next_out = output;
    ret = deflate(&strm, Z_FINISH);//flush);    /* no bad return value */
    assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
    have = output_size - strm.avail_out;
    (void)deflateEnd(&strm);

    return have;
}

int Compression::decompress(unsigned char* input, uint input_size,
                            unsigned char* output, uint output_size)
{
    int ret;
    uint have;
    z_stream strm;
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
            
    ret = inflateInit(&strm);
            
    if (ret != Z_OK)
        return ret;

    strm.avail_in = input_size;
    strm.next_in = input;

    strm.avail_out = output_size;
    strm.next_out = output;
    ret = inflate(&strm, Z_FINISH);//flush);    /* no bad return value */
    assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
    have = output_size - strm.avail_out;
    (void)inflateEnd(&strm);

    output_size = strm.avail_out;
    
    return have;
}

IntAndDouble::IntAndDouble(int i):i(i),
				  d(static_cast<double>(i)),
				  wasInt(true),
				  wasDouble(false)
{}

IntAndDouble::IntAndDouble(double d):i(static_cast<int>(d)),
				     d(d),
				     wasInt(false),
				     wasDouble(true)
{}

int IntAndDouble::getVal(int) const{return i;}
double IntAndDouble::getVal(double) const{return d;}

bool IntAndDouble::isInt() const {return wasInt;}
bool IntAndDouble::isDouble() const {return wasDouble;}


bool HasStringRepresentation::operator<(const HasStringRepresentation& hsr) const
{
    VERBOSER(5, "Calling LEQ test for HasStringRepresentation.\n");
    
    ostringstream oss;
    oss<<*this;
    ostringstream oss1;
    oss1<<hsr;

    VERBOSER(5, "EQ test between "
	     <<*this<<" and "<<hsr<<" yields :: "<<(oss.str() < oss1.str())<<endl);
    
    return oss.str() < oss1.str();
}

bool HasStringRepresentation::operator==(const HasStringRepresentation& hsr) const
{
    VERBOSER(5, "Calling EQ test for HasStringRepresentation.\n");
    ostringstream oss;
    oss<<*this;
    ostringstream oss1;
    oss1<<hsr;

    VERBOSER(5, "EQ test between "
	     <<*this<<" and "<<hsr<<" yields :: "<<(oss.str() == oss1.str())<<endl);
    
    return oss.str() == oss1.str();
}

ostream& operator<<(ostream& o,
		    const HasStringRepresentation& hasStringRepresentation)
{
    if(!hasStringRepresentation.computedAsString){
	hasStringRepresentation.computeAsString("");
    }
    
    return o<<hasStringRepresentation.asString;
}
