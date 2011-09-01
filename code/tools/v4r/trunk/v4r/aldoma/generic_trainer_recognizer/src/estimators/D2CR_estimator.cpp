#include "estimators/D2CR_estimator.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <vector>
#include <pcl/common/time.h>
typedef pcl::KdTree<pcl::PointNormal>::Ptr KdTreePtr;
using namespace std;
using namespace pcl;
template<typename PointInT, typename PointOutT, typename FeatureT>
void
D2CR_Estimator<PointInT, PointOutT, FeatureT>::
D2CR_old (pcl::PointCloud<pcl::PointXYZ> &pc, vector<float> &hist)
{
	const int binsize = 64;
	unsigned int sample_size=40000;
	srand((unsigned)time(0));
	int maxindex = pc.points.size();
	int index1, index2;
	vector<float> dv;
	vector<int> wt;
	dv.reserve(sample_size);
	wt.reserve(sample_size);

    float h_in[binsize] = {0};
    float h_out[binsize] = {0};
    float h_mix[binsize] = {0};
    float h_mix_ratio[binsize] = {0};
    float h_full[binsize/2] = {0};
    float h_fulld[binsize*2] = {0};

	float maxd=0;
	float incnt=0;
	float outcnt=0;
	float mixcnt=0;
	double ratio=0.0;
	for (size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx )
	{
		index1 = rand()%maxindex;
		index2 = rand()%maxindex;
		if (index1==index2)
			continue;
		dv[nn_idx] = pcl::euclideanDistance(pc.points[index1], pc.points[index2]);
		if ( dv[nn_idx] > maxd)
			maxd = dv[nn_idx];

		{
    		int xs = pc.points[index1].x<0.0? floor(pc.points[index1].x)+GRIDSIZE_H : ceil(pc.points[index1].x)+GRIDSIZE_H-1;
    		int ys = pc.points[index1].y<0.0? floor(pc.points[index1].y)+GRIDSIZE_H : ceil(pc.points[index1].y)+GRIDSIZE_H-1;
    		int zs = pc.points[index1].z<0.0? floor(pc.points[index1].z)+GRIDSIZE_H : ceil(pc.points[index1].z)+GRIDSIZE_H-1;
    		int xt = pc.points[index2].x<0.0? floor(pc.points[index2].x)+GRIDSIZE_H : ceil(pc.points[index2].x)+GRIDSIZE_H-1;
    		int yt = pc.points[index2].y<0.0? floor(pc.points[index2].y)+GRIDSIZE_H : ceil(pc.points[index2].y)+GRIDSIZE_H-1;
    		int zt = pc.points[index2].z<0.0? floor(pc.points[index2].z)+GRIDSIZE_H : ceil(pc.points[index2].z)+GRIDSIZE_H-1;
    		wt[nn_idx] = this->lci(xs,ys,zs,xt,yt,zt,ratio);
    		if (wt[nn_idx] == 0) incnt++;
    		if (wt[nn_idx] == 1) outcnt++;
    		if (wt[nn_idx] == 2)
    		{
    			mixcnt++;
    			h_mix_ratio[(int)round(ratio * (binsize-1))]++;
    		}
		}
	}

	for (size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx )
	{
		int idx = (int)round ( dv[nn_idx] / maxd * (binsize/2-1) );
		if (idx < 0 || idx >= binsize/2)
			std::cout << "D2CR_indexfehler" << idx << std::endl;
		else
			h_full[idx]++ ;
		idx = (int)round ( dv[nn_idx] / maxd * (binsize*2-1) );
		if (idx >=0 && idx < binsize*2)
			h_fulld[idx]++ ;
		else
			continue;

		if (wt[nn_idx] == 0)
		{
			h_in[(int)round ( dv[nn_idx] / maxd * (binsize-1) )]++ ;
		}
		if (wt[nn_idx] == 1)
		{
			h_out[(int)round ( dv[nn_idx] / maxd * (binsize-1) )]++;
		}
		if (wt[nn_idx] == 2)
		{
			h_mix[(int)round ( dv[nn_idx] / maxd * (binsize-1) )]++ ;
		}
	}

	for (int i =0; i<binsize/2; i++)
		hist.push_back(h_full[i]);
	for (int i =0; i<binsize*2; i++)
		hist.push_back(h_fulld[i]);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_in[i]);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_out[i]);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_mix[i]);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_mix_ratio[i]);

	float sm = 0;
	for (int i =0; i<hist.size(); i++)
		sm += hist[i];

	for (int i =0; i<hist.size(); i++)
		hist[i] /= sm;


}

template<typename PointInT, typename PointOutT, typename FeatureT>
void
D2CR_Estimator<PointInT, PointOutT, FeatureT>::
D2CR (pcl::PointCloud<pcl::PointXYZ> &pc, vector<float> &hist)
{
	const int binsize = 64;
	const int binsize_h = binsize / 2;
	unsigned int sample_size=pc.points.size() * 10;
	srand((unsigned)time(0));
	int maxindex = pc.points.size();
	int index1, index2;

    float h_in[binsize] = {0};
    float h_out[binsize] = {0};
    float h_mix[binsize] = {0};
    float h_mix_ratio[binsize] = {0};
    float h_full[binsize/2] = {0};
    float h_fulld[binsize*2] = {0};

	double ratio=0.0;
	int vxlcnt=0;
	int inoutmix;
	float dist;
	float didigri;
	for (size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx )
	{
		index1 = rand()%maxindex;
		index2 = rand()%maxindex;
		dist = pcl::euclideanDistance(pc.points[index1], pc.points[index2]);
		didigri = dist / GRIDSIZE;

		h_full[(int)round ( didigri * (binsize_h-1) )]++ ;
		h_fulld[(int)round ( didigri * (binsize*2-1) )]++ ;

		{
    		int xs = pc.points[index1].x<0.0? floor(pc.points[index1].x)+GRIDSIZE_H : ceil(pc.points[index1].x)+GRIDSIZE_H-1;
    		int ys = pc.points[index1].y<0.0? floor(pc.points[index1].y)+GRIDSIZE_H : ceil(pc.points[index1].y)+GRIDSIZE_H-1;
    		int zs = pc.points[index1].z<0.0? floor(pc.points[index1].z)+GRIDSIZE_H : ceil(pc.points[index1].z)+GRIDSIZE_H-1;
    		int xt = pc.points[index2].x<0.0? floor(pc.points[index2].x)+GRIDSIZE_H : ceil(pc.points[index2].x)+GRIDSIZE_H-1;
    		int yt = pc.points[index2].y<0.0? floor(pc.points[index2].y)+GRIDSIZE_H : ceil(pc.points[index2].y)+GRIDSIZE_H-1;
    		int zt = pc.points[index2].z<0.0? floor(pc.points[index2].z)+GRIDSIZE_H : ceil(pc.points[index2].z)+GRIDSIZE_H-1;
    		inoutmix = this->lci(xs,ys,zs,xt,yt,zt,ratio);

			if ( inoutmix == 0 )
			{
				h_in[(int)round ( didigri * (binsize-1) )]++ ;
			}
			if ( inoutmix == 1 )
			{
				h_out[(int)round ( didigri * (binsize-1) )]++;
			}
			if ( inoutmix == 2 )
			{
				h_mix[(int)round ( didigri * (binsize-1) )]++ ;
    			h_mix_ratio[(int)round(ratio * (binsize-1))]++;
			}
		}
	}

	for (int i =0; i<binsize/2; i++)
		hist.push_back(h_full[i]);
	for (int i =0; i<binsize*2; i++)
		hist.push_back(h_fulld[i]);

	for (int i =0; i<binsize; i++)
		hist.push_back(h_in[i]);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_out[i]);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_mix[i]);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_mix_ratio[i]);

	float sm = 0;
	for (int i =0; i<hist.size(); i++)
		sm += hist[i];

	for (int i =0; i<hist.size(); i++)
		hist[i] /= sm;




}

template<typename PointInT, typename PointOutT, typename FeatureT>
int
D2CR_Estimator<PointInT, PointOutT, FeatureT>::
lci(int x1, int y1, int z1, int x2, int y2, int z2, double &ratio)
{
	int voxelcount = 0;
	int voxel_in = 0;
	int act_voxel[3];
	act_voxel[0] = x1;
	act_voxel[1] = y1;
	act_voxel[2] = z1;
	int x_inc, y_inc, z_inc;
	int dx = x2 - x1;
	int dy = y2 - y1;
	int dz = z2 - z1;
    if (dx < 0)
    	x_inc = -1;
    else
        x_inc = 1;
    int l = abs(dx);
    if (dy < 0)
    	y_inc = -1 ;
    else
        y_inc = 1;
    int m = abs(dy);
    if (dz < 0)
    	z_inc = -1 ;
    else
        z_inc = 1;
    int n = abs(dz);
    int dx2 = 2*l;
    int dy2 = 2*m;
    int dz2 = 2*n;
    if ((l >= m) & (l >= n))
    {
        int err_1 = dy2 - l;
        int err_2 = dz2 - l;
        for (int i = 1; i<l; i++)
        {
        	voxelcount++;;
        	if (this->lut[act_voxel[0]][act_voxel[1]][act_voxel[2]] == true)
        		voxel_in++;
            if (err_1 > 0)
            {
            	act_voxel[1] += y_inc;
                err_1 -=  dx2;
            }
            if (err_2 > 0)
            {
            	act_voxel[2] += z_inc;
                err_2 -= dx2;
            }
            err_1 += dy2;
            err_2 += dz2;
            act_voxel[0] += x_inc;


        }

    }
    else if ((m >= l) & (m >= n))
    {
        int err_1 = dx2 - m;
        int err_2 = dz2 - m;
        for (int i=1; i<m; i++)
        {
        	voxelcount++;
        	if (this->lut[act_voxel[0]][act_voxel[1]][act_voxel[2]] == true)
        		voxel_in++;
            if (err_1 > 0)
            {
            	act_voxel[0] +=  x_inc;
                err_1 -= dy2;
            }
            if (err_2 > 0)
            {
            	act_voxel[2] += z_inc;
                err_2 -= dy2;
            }
            err_1 += dx2;
            err_2 += dz2;
            act_voxel[1] += y_inc;
        }
    }
    else
    {
        int err_1 = dy2 - n;
        int err_2 = dx2 - n;
        for (int i=1; i<n; i++)
        {
        	voxelcount++;
        	if (this->lut[act_voxel[0]][act_voxel[1]][act_voxel[2]] == true)
        		voxel_in++;
            if (err_1 > 0)
            {
            	act_voxel[1] += y_inc;
                err_1 -= dz2;
            }
            if (err_2 > 0)
            {
            	act_voxel[0] += x_inc;
                err_2 -= dz2;
            }
            err_1 += dy2;
            err_2 += dx2;
            act_voxel[2] += z_inc;
        }
    }
	voxelcount++;
 	if (this->lut[act_voxel[0]][act_voxel[1]][act_voxel[2]] == true)
		voxel_in++;

	if (voxel_in >=  voxelcount-1)
		return 0;

	if (voxel_in <= 7)
		return 1;

	ratio =  voxel_in / (double)voxelcount;
	return 2;
}

template<typename PointInT, typename PointOutT, typename FeatureT>
void
D2CR_Estimator<PointInT, PointOutT, FeatureT>::
voxelize9(pcl::PointCloud<pcl::PointXYZ> &cluster)
{
	int xi,yi,zi,xx,yy,zz;
	for (size_t i = 0; i < cluster.points.size (); ++i)
	{
		xx = cluster.points[i].x<0.0? floor(cluster.points[i].x)+GRIDSIZE_H : ceil(cluster.points[i].x)+GRIDSIZE_H-1;
		yy = cluster.points[i].y<0.0? floor(cluster.points[i].y)+GRIDSIZE_H : ceil(cluster.points[i].y)+GRIDSIZE_H-1;
		zz = cluster.points[i].z<0.0? floor(cluster.points[i].z)+GRIDSIZE_H : ceil(cluster.points[i].z)+GRIDSIZE_H-1;

		for (int x = -1; x < 2; x++)
    		for (int y = -1; y < 2; y++)
        		for (int z = -1; z < 2; z++)
        		{
        			xi = xx + x;
        			yi = yy + y;
        			zi = zz + z;

        			if (yi >= GRIDSIZE || xi >= GRIDSIZE || zi>=GRIDSIZE || yi < 0 || xi < 0 || zi < 0)
            	    {
            	    	;//ROS_WARN ("[xx][yy][zz] : %d %d %d ",xi,yi,zi);
            	    }
            	    else
            	    	this->lut[xi][yi][zi] = true;
        		}
	}
}


template<typename PointInT, typename PointOutT, typename FeatureT>
void
D2CR_Estimator<PointInT, PointOutT, FeatureT>::
cleanup9(pcl::PointCloud<pcl::PointXYZ> &cluster)
{
	int xi,yi,zi,xx,yy,zz;
	for (size_t i = 0; i < cluster.points.size (); ++i)
	{
		xx = cluster.points[i].x<0.0? floor(cluster.points[i].x)+GRIDSIZE_H : ceil(cluster.points[i].x)+GRIDSIZE_H-1;
		yy = cluster.points[i].y<0.0? floor(cluster.points[i].y)+GRIDSIZE_H : ceil(cluster.points[i].y)+GRIDSIZE_H-1;
		zz = cluster.points[i].z<0.0? floor(cluster.points[i].z)+GRIDSIZE_H : ceil(cluster.points[i].z)+GRIDSIZE_H-1;

		for (int x = -1; x < 2; x++)
    		for (int y = -1; y < 2; y++)
        		for (int z = -1; z < 2; z++)
        		{
        			xi = xx + x;
        			yi = yy + y;
        			zi = zz + z;

        			if (yi >= GRIDSIZE || xi >= GRIDSIZE || zi>=GRIDSIZE || yi < 0 || xi < 0 || zi < 0)
            	    {
            	    	;//ROS_WARN ("[xx][yy][zz] : %d %d %d ",xi,yi,zi);
            	    }
            	    else
            	    	this->lut[xi][yi][zi] = false;
        		}
	}
    //ROS_WARN ("Spent %f seconds in 'cleanup'.", (ros::Time::now () - t1).toSec ());
}


template<typename PointInT, typename PointOutT, typename FeatureT>
void
D2CR_Estimator<PointInT, PointOutT, FeatureT>::
scale_points_unit_cube (pcl::PointCloud<pcl::PointXYZ> &pc, float scalefactor)
{
 	pcl::PointXYZ bmin, bmax;
 	Eigen::Vector4f centroid;
 	pcl::compute3DCentroid 	( pc, centroid);
 	pcl::demeanPointCloud(pc, centroid, pc);
 	Eigen::Vector4f minp,maxp;
 	pcl::getMinMax3D(pc,minp,maxp);
 	double scale_factor = 1;
	vector<float> v;
	v.push_back(abs(maxp[0]));
	v.push_back(abs(maxp[1]));
	v.push_back(abs(maxp[2]));
	v.push_back(abs(minp[0]));
	v.push_back(abs(minp[1]));
	v.push_back(abs(minp[2]));
	sort(v.begin(),v.end());
	scale_factor = 1.0 / v[5] * scalefactor;
 	Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
	matrix.scale(scale_factor);
	pcl::transformPointCloud(pc,pc,matrix);
 }

template<typename PointInT, typename PointOutT, typename FeatureT>
void
D2CR_Estimator<PointInT, PointOutT, FeatureT>::
scale_points_unit_sphere (pcl::PointCloud<pcl::PointXYZ> &pc, float scalefactor)
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid 	( pc, centroid);
	pcl::demeanPointCloud(pc, centroid, pc);

	float max_distance=0, d;
	pcl::PointXYZ cog(0,0,0);

	for (size_t i = 0; i < pc.points.size (); ++i)
	{
		d = pcl::euclideanDistance(cog,pc.points[i]);
		if ( d > max_distance )
			max_distance = d;
	}

	double scale_factor = 1.0 / max_distance * scalefactor;

	Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
	matrix.scale(scale_factor);
    pcl::transformPointCloud(pc,pc,matrix);
}





template<typename PointInT, typename PointOutT, typename FeatureT>
void
D2CR_Estimator<PointInT, PointOutT, FeatureT>::estimate (
                                                         PointInTPtr & in,
                                                         PointOutTPtr & out,
                                                         std::vector<pcl::PointCloud<FeatureT>,
                                                             Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
                                                         std::vector<Eigen::Vector3f> & centroids)
{
  pcl::PointCloud < FeatureT > signature;
  signature.points.resize (1);
  signature.width = 1;
  signature.height = 1;

  /*pcl::visualization::PCLVisualizer vis4 ("scene");
   vis4.addPointCloud<PointInT> (in);
   pcl::visualization::PointCloudColorHandlerCustom<PointOutT> handler (out, 255, 0, 0);
   vis4.addPointCloud<PointOutT> (out,handler,"out");
   vis4.spin();*/

  pcl::copyPointCloud(*in,*out);

  Eigen::Vector4f centroid4f;
  pcl::compute3DCentroid (*in, centroid4f);
  Eigen::Vector3f centroid3f (centroid4f[0], centroid4f[1], centroid4f[2]);
  centroids.push_back (centroid3f);

  std::cout << "D2CR_Estimator::estimate()" << std::endl;
  // #################################################### ..compute D2CR
  vector<float> hist;
  scale_points_unit_sphere (*in, GRIDSIZE_H); //TODO: Should not modify the pointcloud!!
  {
	  ScopeTime t ("-------- D2CR_Estimator::estimate()");
		this->voxelize9 (*in);
		this->D2CR (*in, hist);
		this->cleanup9 (*in);
  }
  //pcl::io::savePCDFileASCII("wwcluster.pcd",*in);

  signatures.resize (1);
  for (int i = 0; i < 416; i++)
    signature.points[0].histogram[i] = hist[i];
  signatures[0] = (signature);
  //pcl::io::savePCDFileASCII("d.pcd",signatures[0]);

  pcl::copyPointCloud(*out,*in);
}

template class D2CR_Estimator<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<416> > ;
template class D2CR_Estimator<pcl::PointXYZ, pcl::PointXYZ, pcl::Histogram<416> > ;
