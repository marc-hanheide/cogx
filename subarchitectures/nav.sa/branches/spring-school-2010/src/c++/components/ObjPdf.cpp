#include "ObjPdf.hpp"

ObjPdf::ObjPdf(int gridsize)
{
	pdfgrid = new double* [gridsize];
	for(int i=0;i<gridsize;i++){
		*(pdfgrid+i)=new double[gridsize];
	}
	m_gridsize = gridsize;
}

ObjPdf::~ObjPdf()
{
}
