

#ifndef _HIST_DESC_H_
#define _HIST_DESC_H_

#include <v4r/TomGine/tgHistogram.h>
#include <v4r/TomGine/tgModel.h>

/** @brief Descriptor of image patch using histograms. */
class HistDesc
{
public:
	std::vector< TomGine::tgHistogram > m_histlist;
	std::vector< std::vector<double> > m_subdata;
	std::vector< float > m_maskratio;
	TomGine::tgRect2Di m_bb;
	unsigned m_subsize;
	unsigned m_nsegx, m_nsegy;

	void Preprocess(	const std::vector<double> &data,
						const TomGine::tgRect2Di &bb,
						unsigned subsize );

public:

	void Reset();

	/** @brief Calculates the histograms of the descriptor.
	 *  @param data		The data block of the image patch.	 */
	void Process( 	const std::vector<double> &data,
					double min_val, double max_val,
					const TomGine::tgRect2Di &bb,
					unsigned subsize );

	/** @brief Calculates the histograms of the descriptor with object mask.
	 *  @param data		The data block of the image patch.
	 *  @param mask		The mask indicating if a data element belongs to the object or not. (must be same size as data)*/
	void Process(	const std::vector<double> &data,
					double min_val, double max_val,
					const TomGine::tgRect2Di &bb,
					unsigned subsize,
					const std::vector<bool> &mask, double vis_ratio=0.5 );


	double IntersectionMatch(const HistDesc& h, double vis_ratio=0.5) const;

	/** @brief Inner product of histograms of two descriptors. */
	std::vector<double> operator*(const HistDesc& h) const;

	/** @brief Euclidean distance of histograms of two descriptors. */
	std::vector<double> Euclidean(const HistDesc& h) const;

	/** @brief Intersection of histograms of two descriptors. */
	std::vector<double> Intersect(const HistDesc& h) const;

	/** @brief Fidelity similarity of histograms of two descriptors. */
	std::vector<double> Fidelity(const HistDesc& h) const;

};

#endif
