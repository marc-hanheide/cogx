
#include "HistDesc.h"

bool ispow2(unsigned a)
{
	unsigned i=1;
	while(a>pow(2,i))
		i++;

	return a==pow(2,i);
}

unsigned global2local(	unsigned idx,
						const TomGine::tgRect2Di &bb,
						const unsigned &subsize,
						const unsigned &nsegx,
						unsigned &subidx)
{
	unsigned I = idx % bb.w;
	unsigned J = idx / bb.w;

	unsigned i = I % subsize;
	unsigned j = J % subsize;

	unsigned segx = I / subsize;
	unsigned segy = J / subsize;

	subidx = segy*nsegx + segx;

//	printf("%d, %d %d, %d %d, %d, r: %d %d\n", idx, I, J, i, j, subidx, bb.w, bb.h);

	return (j*subsize + i);
}

unsigned local2global(	unsigned idx,
						const TomGine::tgRect2Di &bb,
						const unsigned &subsize,
						const unsigned &segx, const unsigned &segy)
{
	unsigned i = idx % subsize;
	unsigned j = idx / subsize;

	unsigned I = subsize * segx + i;
	unsigned J = subsize * segy + j;

	return (J*bb.w + I);
}

void HistDesc::Preprocess(	const std::vector<double> &data,
							const TomGine::tgRect2Di &bb,
							unsigned subsize )
{
	if(!ispow2(bb.w)){
		printf("[HistDesc::Preprocess] Warning width of bounding-box not power of 2.\n");
		return;
	}
	if(!ispow2(bb.h)){
		printf("[HistDesc::Preprocess] Warning height of bounding-box not power of 2.\n");
		return;
	}
	if(!ispow2(subsize)){
		printf("[HistDesc::Preprocess] Warning size of sub-region not power of 2.\n");
		return;
	}
	if( bb.w<subsize || bb.h<subsize){
		printf("[HistDesc::Preprocess] Warning size of bounding-box smaller than size of sub-region.\n");
		return;
	}

	m_subsize = subsize;
	m_bb = bb;

	m_nsegx = m_bb.w / m_subsize;
	m_nsegy = m_bb.h / m_subsize;

	if(data.size()< m_bb.w*m_bb.h){
		printf("[HistDesc::Preprocess] Warning size of data to small [%d < %dx%d].\n", (int)data.size(), m_bb.w, m_bb.h);
		return;
	}

}

void HistDesc::Reset()
{
	unsigned size = m_nsegx*m_nsegy;

	m_histlist.assign( size, TomGine::tgHistogram(32) );

	m_subdata.assign( size, std::vector<double>() );

	for(unsigned i=0; i<m_subdata.size(); i++)
		m_subdata[i].assign(m_subsize*m_subsize, 0.0f);

	m_maskratio.assign( size, 0.0f );

}

void HistDesc::Process( const std::vector<double> &data,
						double min_val, double max_val,
						const TomGine::tgRect2Di &bb,
						unsigned subsize)
{

	Preprocess(data, bb, subsize);

	printf("[HistDesc::Process] m_nsegx: %d, m_nsegy: %d\n", m_nsegx, m_nsegy);

	Reset();

	unsigned subidx=0;
	for(unsigned IDX=0; IDX<data.size(); IDX++)
	{
		float d = data[IDX];
		unsigned idx = global2local(IDX, m_bb, m_subsize, m_nsegx, subidx);
		m_subdata[subidx][idx] = d;
	}

	for( subidx=0; subidx<m_subdata.size(); subidx++ ){
		m_histlist[subidx].Evaluate(m_subdata[subidx],min_val,max_val);
	}


//	for( subidx=0; subidx<m_subdata.size(); subidx++ ){
//		for(unsigned j=0; j<m_subsize; j++){
//			for(unsigned i=0; i<m_subsize; i++)
//				printf("%.1f ", m_subdata[subidx][j*m_subsize+i]);
//			printf("\n");
//		}
//		printf("\n");
//	}
//	printf("min, max: %f %f\n", min_val, max_val);

}

void HistDesc::Process( const std::vector<double> &data,
						double min_val, double max_val,
						const TomGine::tgRect2Di &bb,
						unsigned subsize,
						const std::vector<bool> &mask, double vis_ratio)
{
	Preprocess(data, bb, subsize);

//	printf("[HistDesc::HistDesc] m_nsegx: %d, m_nsegy: %d\n", m_nsegx, m_nsegy);

	Reset();


	double maskdiv = 1.0/(m_subsize*m_subsize);
	unsigned subidx=0;

	for(unsigned IDX=0; IDX<data.size(); IDX++)
	{
		float d = data[IDX];
		unsigned idx = global2local(IDX, m_bb, m_subsize, m_nsegx, subidx);
//		printf("%d %d\n", subidx, idx);
		m_subdata[subidx][idx] = d;

		if(mask[IDX])
			m_maskratio[subidx] += maskdiv;
	}

	for( unsigned i=0; i<m_subdata.size(); i++ ){
		if(this->m_maskratio[i] > vis_ratio)
			m_histlist[i].Evaluate(m_subdata[i],min_val,max_val);
	}

//	for( subidx=0; subidx<m_subdata.size(); subidx++ ){
//		printf("%f\n", m_maskratio[subidx]);
//		for(unsigned j=0; j<m_subsize; j++){
//			for(unsigned i=0; i<m_subsize; i++)
//				printf("%.1f ", m_subdata[subidx][j*m_subsize+i]);
//			printf("\n");
//		}
//		printf("\n");
//	}
//	printf("min, max: %f %f\n", min_val, max_val);

}

double HistDesc::IntersectionMatch(const HistDesc& h, double vis_ratio) const
{
	std::vector<double> result;
	double val, mean(0.0);
	if( this->m_histlist.size() != h.m_histlist.size() )	{
		printf("[HistDesc::Intersect] Number of descriptor histograms do not match.\n");
		return 0.0;
	}

	for( unsigned i=0; i<m_subdata.size(); i++ ){
		if(this->m_maskratio[i] > vis_ratio){
			val =  this->m_histlist[i].Intersect(h.m_histlist[i]);
			result.push_back( val );
			mean += val;
		}
	}

	mean /= result.size();

	double var=0.0;
	for(unsigned i=0; i<result.size(); i++)
		var += pow(result[i] - mean,2);
	var = var / result.size();

	return var;
}

std::vector<double> HistDesc::operator*(const HistDesc& h) const
{
	std::vector<double> result;
	if( this->m_histlist.size() != h.m_histlist.size() )	{
		printf("[HistDesc::operator*] Number of descriptor histograms do not match.\n");
		return result;
	}

	for( unsigned i=0; i<m_subdata.size(); i++ ){
		result.push_back( this->m_histlist[i] * h.m_histlist[i] );
	}
	return result;
}

std::vector<double> HistDesc::Euclidean(const HistDesc& h) const
{
	std::vector<double> result;
	if( this->m_histlist.size() != h.m_histlist.size() )	{
		printf("[HistDesc::Euclidean] Number of descriptor histograms do not match.\n");
		return result;
	}

	for( unsigned i=0; i<m_subdata.size(); i++ ){
		result.push_back( this->m_histlist[i].Euclidean(h.m_histlist[i]) );
	}
	return result;
}

std::vector<double> HistDesc::Intersect(const HistDesc& h) const
{
	std::vector<double> result;
	if( this->m_histlist.size() != h.m_histlist.size() )	{
		printf("[HistDesc::Intersect] Number of descriptor histograms do not match.\n");
		return result;
	}

	for( unsigned i=0; i<m_subdata.size(); i++ ){
		result.push_back( this->m_histlist[i].Intersect(h.m_histlist[i]) );
	}
	return result;
}

std::vector<double> HistDesc::Fidelity(const HistDesc& h) const
{
	std::vector<double> result;
	if( this->m_histlist.size() != h.m_histlist.size() )	{
		printf("[HistDesc::Fidelity] Number of descriptor histograms do not match.\n");
		return result;
	}

	for( unsigned i=0; i<m_subdata.size(); i++ ){
		result.push_back( this->m_histlist[i].Fidelity(h.m_histlist[i]) );
	}
	return result;
}
