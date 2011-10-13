/**
 * $Id$
 * Johann Prankl, 2011-02-24
 * prankl@acin.tuwien.ac.at
 */


#include "VocabularyTree.hh"

#define DEBUG


namespace P 
{

unsigned VocabularyTree::nbcnt=0;

static const bool CmpMatchesDec(const cv::Ptr<OVMatches>& a, const cv::Ptr<OVMatches>& b)
{
  return (a->conf > b->conf);
}


VocabularyTree::VocabularyTree(cv::Ptr<cv::DescriptorMatcher> &descMatcher, Parameter _param)
 : N(0), haveNorm(0), param(_param)
{
  matcher = descMatcher;
  root = new VTNode(N);
}

VocabularyTree::~VocabularyTree()
{
}




/************************************** PRIVATE ************************************/

/**
 * CreateNodes
 */
void VocabularyTree::CreateNodes(VTNode &node, vector<float*> &ptDescs, unsigned &sizeDescs, unsigned &l)
{
  if (ptDescs.size() > param.k)
  {
    descs = cv::Mat_<float>(ptDescs.size(), sizeDescs); 
    for (unsigned i=0; i<ptDescs.size(); i++)
    {
      float *ds = ptDescs[i];
      float *dd = &descs(i,0);
      for (unsigned j=0; j<sizeDescs; j++, ds++, dd++)
        *dd = *ds;
    }

    cout<<".";
    cv::kmeans(descs, param.k, labels,
          cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 0.001 ),
          param.numRandCenters, param.flags, node.descriptors);

    vector< vector<float*> > ptDescsBranches(param.k);
    for (unsigned i=0; i<labels.rows; i++)
      ptDescsBranches[labels(i,0)].push_back(ptDescs[i]);

    ptDescs.clear();
    ptDescs.reserve(0);
    
    l++;
    node.nodes.resize(ptDescsBranches.size());
    for (unsigned i=0; i<ptDescsBranches.size(); i++)
    {
      node.nodes[i] = new VTNode(N);

      if (l<param.l)
      {
        CreateNodes(*node.nodes[i], ptDescsBranches[i], sizeDescs, l);
      }
    }
    l--;
    
  }
  else cout<<endl<<"Run out of descriptors!"<<endl;
}


/**
 * write nodes recursivly
 */
void VocabularyTree::Write(ofstream &os, VTNode &node)
{
  //save descriptors
  os<<node.descriptors.rows<<' '<<node.descriptors.cols<<'\n';
  float *d = node.descriptors.ptr<float>();
  for (unsigned i=0; i<node.descriptors.rows*node.descriptors.cols; i++,d++)
    os<<(*d)<<' ';
  os<<'\n';

  //save child nodes
  os<<node.nodes.size()<<'\n';
  for (unsigned i=0; i<node.nodes.size(); i++)
    Write(os,*node.nodes[i]);
}

/**
 * read nodes recursivly
 */
void VocabularyTree::Read(ifstream &is, VTNode &node)
{
  //load descriptors
  unsigned nbNodes, rows, cols;

  is>>rows>>cols;
  if (rows>0 && cols>0)
  {
    node.descriptors = cv::Mat_<float>(rows,cols);
    float *d = node.descriptors.ptr<float>();

    for (unsigned i=0; i<node.descriptors.rows*node.descriptors.cols; i++,d++)
      is>>(*d);
  }

  //save child nodes
  is>>nbNodes;
  node.nodes.resize(nbNodes);
  for (unsigned i=0; i<node.nodes.size(); i++)
  {
    node.nodes[i] = new VTNode(N);
    Read(is,*node.nodes[i]);
  }
}

/**
 * Assign a keypoint to the tree
 * TODO: do not insert to root node!!
 */
void VocabularyTree::AssignDescriptor(VTNode &node, cv::Mat_<float> &desc, unsigned vidx, unsigned kidx, vector<cv::DMatch> &tmp)
{
  //node.Insert(oidx,vidx,kidx);

  if (node.nodes.size()>0)
  {
    matcher->match(desc,node.descriptors, tmp);
    node.nodes[tmp[0].trainIdx]->Insert(vidx,kidx);
    AssignDescriptor(*node.nodes[tmp[0].trainIdx],desc,vidx,kidx,tmp);
  }
}

/**
 * QueryDescriptor
 */
void VocabularyTree::QueryDescriptor(cv::Ptr<VTNode> &node, cv::Mat_<float> &desc, vector<short> &path, cv::Ptr<VTNode> &matchNode, vector<cv::DMatch> &tmp)
{
  node->ni++;
  if (node->nodes.size()==0)
  {
    matchNode = node;
  }
  else
  {
    matcher->match(desc,node->descriptors, tmp);
    path.push_back(tmp[0].trainIdx);
    QueryDescriptor(node->nodes[tmp[0].trainIdx],desc,path, matchNode,tmp);
  }
}

/**
 * Parse graph and set ni=0
 */
void VocabularyTree::Reset_ni(VTNode &node)
{
  node.ni=0;
  for (unsigned i=0; i<node.nodes.size(); i++)
    Reset_ni(*node.nodes[i]);
}

/**
 * sum up to norm2
 */
void VocabularyTree::Norm2_n(VTNode &node, float &norm_n)
{
  norm_n += PMath::Sqr(node.ni*node.wi);
  for (unsigned i=0; i<node.nodes.size(); i++)
    Norm2_n(*node.nodes[i], norm_n);
}

/**
 * sum up to norm2
 */
void VocabularyTree::Norm2_m(VTNode &node, vector<float> &norm_m)
{
  map<unsigned,vector<unsigned> >::iterator it;

  for (it=node.links.begin(); it!=node.links.end(); it++)
    norm_m[it->first] += PMath::Sqr(it->second.size()*node.wi);

  for (unsigned i=0; i<node.nodes.size(); i++)
    Norm2_m(*node.nodes[i], norm_m);
}


/**
 * compute the normalized differences
 */
void VocabularyTree::SumDiff(VTNode &node, vector<short> &path, int &z, vector<float> &norm_m, float norm_n, vector<pair<unsigned,float> > &diff)
{
  z++;
  if (node.nb!=nbcnt)
  {
    node.nb=nbcnt;

    float nqi = node.ni*node.wi/norm_n;
    map<unsigned,vector<unsigned> >::iterator it;

    for (it=node.links.begin(); it!=node.links.end(); it++)
    {
      if (!PMath::IsZero(norm_m[it->first]))
        diff[it->first].second = diff[it->first].second + nqi*(it->second.size()*node.wi / norm_m[it->first]);
    }
  }

  if (z<path.size()) SumDiff(*node.nodes[path[z]],path,z,norm_m,norm_n, diff);
  z--;
}

/**
 * clear all nodes, i.e. delete all inserted views
 */
void VocabularyTree::Clear(VTNode &node)
{
  node.nb = 0;
  node.ni = 0;
  node.wi = 0;
  node.links.clear();

  for (unsigned i=0; i<node.nodes.size(); i++)
    Clear(*node.nodes[i]);
}





/************************************** PUBLIC ************************************/
/**
 * Clear the tree and delete all view links
 */
void VocabularyTree::clear()
{ 
  N = 0;
  views.clear();  
  haveNorm = false;
  norm_m.clear();
  diff.clear();

  Clear(*root);
}

/**
 * Query an object view
 */
void VocabularyTree::QueryObjects(cv::Mat_<float> &descriptors, vector<cv::Ptr<OVMatches> > &matches)
{
  matches.clear();
  vector<cv::DMatch> tmp;
  vector< cv::Ptr<VTNode> > matchNodes(descriptors.rows);
  vector< vector<short> > matchPath(descriptors.rows);
  cv::Mat_<float> roiDesc;
  float norm_n=0.;
 
  if (!haveNorm) PrecomputeNorm2_m(); 

  Reset_ni(*root);

  // match
  for (unsigned i=0; i<descriptors.rows; i++)
  {
    roiDesc = cv::Mat_<float>(descriptors,cv::Rect(0,i,descriptors.cols,1));
    QueryDescriptor(root,roiDesc, matchPath[i], matchNodes[i], tmp);
  }

  // sum up weights
  Norm2_n(*root, norm_n);
  if(!PMath::IsZero(norm_n))
  {
    norm_n = sqrt(norm_n);

    for (unsigned i=0; i<diff.size(); i++) diff[i]=pair<unsigned,float>(i,0.);

    nbcnt++;
    for (unsigned i=0; i<descriptors.rows; i++)
    {
      int z=-1;
      SumDiff(*root, matchPath[i], z, norm_m,norm_n,diff);
    }

    for (unsigned i=0; i<diff.size(); i++) diff[i].second = 2.- 2.*diff[i].second;

    //return keypoints
    vector<cv::Ptr<OVMatches> > tmpMatches(diff.size());
    for (unsigned i=0; i<views.size(); i++)
      tmpMatches[i] = new OVMatches(views[i]->idx, views[i]->id, 1.-(diff[i].second/2.));

    map<unsigned,vector<unsigned> >::iterator it;
    for (unsigned i=0; i<matchNodes.size(); i++)
    {
      for (it=matchNodes[i]->links.begin(); it!=matchNodes[i]->links.end(); it++)
      {
        for (unsigned j=0; j<it->second.size(); j++)
          tmpMatches[it->first]->matches.push_back(cv::DMatch(i, it->second[j], views[it->first]->id, 0.));
      }
    }

    //copy back result
    for (unsigned i=0; i<tmpMatches.size(); i++)
      if (tmpMatches[i]->matches.size() > 0 && tmpMatches[i]->conf>0.001)
        matches.push_back(tmpMatches[i]);

    sort(matches.begin(), matches.end(), CmpMatchesDec);
  }
}

/**
 * Normalize the weights
 */
void VocabularyTree::PrecomputeNorm2_m()
{
  norm_m.resize(views.size());
  diff.resize(views.size());
  for (unsigned i=0; i<norm_m.size(); i++) norm_m[i]=0.;

  Norm2_m(*root, norm_m); 
  
  for (unsigned i=0; i<norm_m.size(); i++) 
    if (!PMath::IsZero(norm_m[i])) norm_m[i] = sqrt(norm_m[i]);
  haveNorm=true;
}


/**
 * Insert an object view to the tree structure
 */
void VocabularyTree::InsertView(unsigned oidx, unsigned vidx, cv::Ptr<View> &view)
{
  view->id = vidx;        // that should not be here, but just to be sure...
  view->idx = oidx;
  views.push_back(view);

  N = views.size();
  cv::Mat_<float> roiDesc;

  vector<cv::DMatch> matches;
  haveNorm=false;

  for (unsigned i=0; i<view->descriptors.rows; i++)
  {
    roiDesc = cv::Mat_<float>(view->descriptors,cv::Rect(0,i,view->descriptors.cols,1));
    AssignDescriptor(*root,roiDesc, views.size()-1, i,matches);
  }
}


/**
 * Add descriptors from an object model
 */
void VocabularyTree::AddForClustering(const cv::Ptr<CModel> &model)
{
  for (unsigned i=0; i<model->views.size(); i++)
  {
    descMats.push_back(cv::Mat_<float>());
    model->views[i]->descriptors.copyTo(descMats.back());
  }
}

/**
 * Add a mat of descriptors
 */
void VocabularyTree::AddForClustering(const cv::Mat_<float> &descriptors)
{
  descMats.push_back(cv::Mat_<float>());
  descriptors.copyTo(descMats.back());
}


/**
 * CreateTree
 */
void VocabularyTree::CreateTree()
{
  if (descMats.size()==0)
    throw runtime_error("VocabularyTree::CreateTree : No descriptors to cluster available");

  //count number of descriptors
  unsigned sizeDescs = descMats[0].cols;
  vector<float*> ptDescs;
  for (unsigned i=0; i<descMats.size(); i++)
  {
    for (unsigned j=0; j<descMats[i].rows; j++)
      ptDescs.push_back(&descMats[i](j,0));
  }
  #ifdef DEBUG
  cout<<"Number of keypoints to cluster: "<<ptDescs.size()<<endl;
  #endif

  //create tree
  unsigned l=0;
  root->nodes.clear();

  cout<<"Start clustering";

  CreateNodes(*root, ptDescs, sizeDescs, l);
  
  cout<<endl;
  
  descMats.clear();
}

/**
 * load a tree
 */
void VocabularyTree::Load(const string &filename)
{
  cout<<"Load from file: "<<filename<<"...";
  root->nodes.clear();  

  ifstream in(filename.c_str());
  Read(in,*root);
  in.close();

  cout<<"ok"<<endl;
}

/**
 * save a tree
 */
void VocabularyTree::Save(const string &filename)
{
  cout<<"Save to file: "<<filename<<"...";

  ofstream out(filename.c_str());
  Write(out, *root);  
  out.close();

  cout<<"ok"<<endl;  
}



}







