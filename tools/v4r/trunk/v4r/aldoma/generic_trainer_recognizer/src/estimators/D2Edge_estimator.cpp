#include "estimators/D2Edge_estimator.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <vector>

typedef pcl::KdTree<pcl::PointNormal>::Ptr KdTreePtr;
using namespace std;

/*
// indices is a array same order as points, with 1 being a edge point, 0 not
template<typename PointInT, typename PointOutT, typename FeatureT>
void
D2Edge_Estimator<PointInT, PointOutT, FeatureT>::
get_edges(pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<int> &indices, float curvature_threshold)
{
	// ### do Meshresolution:: calculate leaf_size
	typedef typename pcl::KdTree<PointT>::Ptr KdTreeInPtr;
	KdTreeInPtr tree = boost::make_shared<pcl::KdTreeFLANN<PointT> > (false);
	tree->setInputCloud (cloud);
	//update leaf_size_
	std::vector<int> nn_indices (9);
	std::vector<float> nn_distances (9);
	std::vector<int> src_indices;

	float sum_distances = 0.0;
	std::vector<float> avg_distances (cloud->points.size ());
	float avg_dist_neighbours = 0.0;
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		avg_dist_neighbours = 0.0;
		tree->nearestKSearch (cloud->points[i], 9, nn_indices, nn_distances);
		for (size_t j = 1; j < nn_indices.size (); j++)
			avg_dist_neighbours += sqrt (nn_distances[j]);

		avg_dist_neighbours /= nn_indices.size ();
		avg_distances.push_back(avg_dist_neighbours);
		sum_distances += avg_dist_neighbours;
	}
	float leaf_size = sum_distances / cloud->points.size ();

	leaf_size *= 3;
	// ### calculate Curvature for each point
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (leaf_size);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.compute (*cloud_normals);


	for (size_t i = 0; i < cloud.points.size (); i++)
	{
		if ( cloud_normals->points[i].curvature > curvature_threshold)
			indices[i] = 1;
		else
			indices[i] = 0;
	}

  }

*/

template<typename PointInT, typename PointOutT, typename FeatureT>
void
D2Edge_Estimator<PointInT, PointOutT, FeatureT>::
D2Edge (pcl::PointCloud<pcl::PointXYZ> &pc, vector<float> &hist)
{
/*	const int binsize = 64;
	unsigned int sample_size=40000;
	srand((unsigned)time(0));
	int maxindex = pc.points.size();

	int index1, index2, index3;

    float h_d2_all[binsize] = {0};
    float h_d2_in[binsize] = {0};
    float h_d2_out[binsize] = {0};
    float h_d2_mix[binsize] = {0};
    float h_d2_mix_ratio[binsize] = {0};

    float h_e_full[binsize] = {0};
    float h_e_in[binsize] = {0};
    float h_e_out[binsize] = {0};
    float h_e_mix[binsize] = {0};
    float h_e_mix_ratio[binsize] = {0};

    std::vector<int> edge_points;
    edge_points.reserve(pc.points.size());
    this->get_edges(pc,edge_points,0.5);

	float gsh = GRIDSIZE_H;
	int binsize_h = binsize / 2;
	double ratio1=0,ratio2=0,ratio3=0;
	double pih = M_PI / 2.0;
	float a,b,c,s;
    int th1,th2,th3;
    int vxlcnt = 0;
    int pcnt1,pcnt2,pcnt3;
	int inoutmix;
    float dist;
    float didigri;
    bool is_edge = false;
    for (size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx)
	{
		// get 2 random points
		index1 = rand()%maxindex; index2 = rand()%maxindex;
		if ( index1==index2 ) continue;
		// make sure, both are from edge-points or both are non-edge-points
		if ( edge_points[index1] == edge_points[index2] && edge_points[index1] == 1)
			is_edge = true;
		else
			is_edge = false;

		Eigen::Vector4f p1 = pc.points[index1].getVector4fMap();
		Eigen::Vector4f p2 = pc.points[index2].getVector4fMap();


		didigri = pcl::euclideanDistance(pc.points[index1], pc.points[index2])  / GRIDSIZE;
		// IN, OUT, MIXED, Ratio line tracing, index1->index2
		{
    		int xs = p1[0] < 0.0? floor(p1[0])+GRIDSIZE_H : ceil(p1[0])+GRIDSIZE_H-1;
    		int ys = p1[1] < 0.0? floor(p1[1])+GRIDSIZE_H : ceil(p1[1])+GRIDSIZE_H-1;
    		int zs = p1[2] < 0.0? floor(p1[2])+GRIDSIZE_H : ceil(p1[2])+GRIDSIZE_H-1;
    		int xt = p2[0] < 0.0? floor(p2[0])+GRIDSIZE_H : ceil(p2[0])+GRIDSIZE_H-1;
    		int yt = p2[1] < 0.0? floor(p2[1])+GRIDSIZE_H : ceil(p2[1])+GRIDSIZE_H-1;
    		int zt = p2[2] < 0.0? floor(p2[2])+GRIDSIZE_H : ceil(p2[2])+GRIDSIZE_H-1;
    		inoutmix = this->lci(xs,ys,zs,xt,yt,zt,ratio1) ;

    		if ( is_edge )
    		{

    		}
    		else
    		{
    			h_d2_all[(int)round ( didigri * (binsize-1) )]++ ;
    			if ( inoutmix == 0 )
    				h_d2_in[(int)round ( didigri * (binsize-1) )]++ ;
    			if ( inoutmix == 1 )
    				h_out[(int)round ( didigri * (binsize-1) )]++;
    			if ( inoutmix == 2 )
    			{
    				h_mix[(int)round ( didigri * (binsize_h-1) )]++ ;
        			h_mix_ratio[(int)round(ratio1 * (binsize-1))]++;
    			}

    		}
		}

		// heuristic to decide if in, out mixed... TODO   get something smart...
		// D3 ( herons formula )
		float d3area = ( sqrt(sqrt( s * (s-a) * (s-b) * (s-c) )) ) / maxd3;
		if ( d3area > 1.0)
			cout << "index d3area error\n";


		if (vxlcnt_sum <= 21)
		{
			h_a3_out[th1]++;
			h_a3_out[th2]++;
			h_a3_out[th3]++;
			h_d3_out[(int)round ( d3area * (binsize-1) )]++;
		}
		else
			if ( p_cnt - vxlcnt_sum < 4)
			{
				h_a3_in[th1]++;
				h_a3_in[th2]++;
				h_a3_in[th3]++;
				h_d3_in[(int)round ( d3area * (binsize-1) )]++;
			}
			else
			{
				if ( ratio3 <= .5 )
					h_a3_mix[th1/2]++;
				else
					h_a3_mix[binsize_h + th1/2]++;

				if ( ratio2 <= .5 )
					h_a3_mix[th2/2]++;
				else
					h_a3_mix[binsize_h + th2/2]++;

				if ( ratio1 <= .5 )
					h_a3_mix[th3/2]++;
				else
					h_a3_mix[binsize_h + th3/2]++;

				if ( vxlcnt_sum / p_cnt <= 0.5 )
					h_d3_mix[(int)round ( d3area * (binsize_h-1) )]++;
				else
					h_d3_mix[(int)round ( binsize_h + d3area * (binsize_h-1) )]++;
			}
	}
//		(0.5, 0.5, 0.75, 0.75, 0.75, 0.75, 0.75, 1.5, 1.5, 1.5)
	for (int i =0; i<binsize; i++)
		hist.push_back(h_a3_in[i] * .5);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_a3_out[i] * .5);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_a3_mix[i] * 0.75);

	for (int i =0; i<binsize; i++)
		hist.push_back(h_d3_in[i] * 0.75);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_d3_out[i] * 0.75);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_d3_mix[i]* 0.75);

	for (int i =0; i<binsize; i++)
		hist.push_back(h_in[i] * 0.75);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_out[i] * 1.5);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_mix[i] * 1.5);
	for (int i =0; i<binsize; i++)
		hist.push_back(h_mix_ratio[i]*0.75);

	float sm = 0;
	for (int i =0; i<hist.size(); i++)
		sm += hist[i];

	for (int i =0; i<hist.size(); i++)
		hist[i] /= sm;
*/
}

template<typename PointInT, typename PointOutT, typename FeatureT>
int
D2Edge_Estimator<PointInT, PointOutT, FeatureT>::
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
D2Edge_Estimator<PointInT, PointOutT, FeatureT>::
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
D2Edge_Estimator<PointInT, PointOutT, FeatureT>::
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
D2Edge_Estimator<PointInT, PointOutT, FeatureT>::
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
D2Edge_Estimator<PointInT, PointOutT, FeatureT>::
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
D2Edge_Estimator<PointInT, PointOutT, FeatureT>::estimate (
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

  std::cout << "D2Edge_Estimator::estimate()" << std::endl;
  // #################################################### ..compute D2Edge
  vector<float> hist;
  scale_points_unit_sphere (*in, GRIDSIZE_H); //TODO: Should not modify the pointcloud!!
  this->voxelize9 (*in);
  this->D2Edge (*in, hist);
  this->cleanup9 (*in);
  //pcl::io::savePCDFileASCII("wwcluster.pcd",*in);

  signatures.resize (1);
  for (int i = 0; i < 416; i++)
    signature.points[0].histogram[i] = hist[i];
  signatures[0] = (signature);
  //pcl::io::savePCDFileASCII("d.pcd",signatures[0]);

  pcl::copyPointCloud(*out,*in);
}

template class D2Edge_Estimator<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<416> > ;
template class D2Edge_Estimator<pcl::PointXYZ, pcl::PointXYZ, pcl::Histogram<416> > ;
