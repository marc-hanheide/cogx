#ifndef OBJPDF_HPP_
#define OBJPDF_HPP_

class ObjPdf
{
public:
	ObjPdf(int gridsize);
	virtual ~ObjPdf();
	double** pdfgrid;
	int m_gridsize;
	
};

#endif /*OBJPDF_HPP_*/
