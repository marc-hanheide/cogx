/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */


#include "KeypointDescriptor.hh"




namespace P 
{

float KeypointDescriptor::DOG_SIFT_THR = .2;
float KeypointDescriptor::LOWE_DOG_SIFT_THR = 50000;
float KeypointDescriptor::MSER_SIFT_THR = 40000;
float KeypointDescriptor::HESLAP_SIFT_THR = 30000;

const unsigned KeypointDescriptor::DELETE = 0x01;      //delete occurrence
const unsigned KeypointDescriptor::INSERT = 0x02;      //insert to codebook
const unsigned KeypointDescriptor::DETECTED = 0x04;    //keypoint is detected


static const int NAME_LENGTH = 40;
static const char keyd_type_names[][NAME_LENGTH] = {
  "DOG_SIFT",
  "LOWE_DOG_SIFT",
  "MSER_SIFT",
  "UVMSER_SIFT",
  "HESLAP_SIFT",
  "UNDEF"
  };

/**
 * Returns the name of a given gestalt type.
 */
const char* KeypointDescriptor::TypeName(Type t)
{
  return keyd_type_names[t];
}

/**
 *  * Return the enum type of a given gestalt type name.
 *   */
KeypointDescriptor::Type KeypointDescriptor::EnumType(const char *type_name)
{
  for(int i = 0; i < MAX_TYPE; i++)
    if(strncmp(type_name, keyd_type_names[i], NAME_LENGTH) == 0)
      return (Type)i;
  return UNDEF;
}



/**
 * Constructor/Destructor
 */
KeypointDescriptor::KeypointDescriptor()
 : flags(0),
   type(UNDEF),
   vec(0),
   size(0),
   pos(0),
   patch(0),
   error(0),
   cntError(0)
{
}

KeypointDescriptor::KeypointDescriptor(Type t)
 : flags(0),
   type(t),
   vec(0),
   size(0),
   pos(0),
   patch(0),
   error(0),
   cntError(0)
{
}


KeypointDescriptor::~KeypointDescriptor()
{
  if (vec!=0) delete[] vec;
  if (patch!=0) cvReleaseImage(&patch);
  if (pos!=0) cvReleaseMat(&pos);
}

KeypointDescriptor::KeypointDescriptor(KeypointDescriptor *k)
 : Keypoint(*k),
   flags(0),
   vec(0), 
   size(0),
   pos(0),
   patch(0)
{
  Copy(k);
}

KeypointDescriptor::KeypointDescriptor(Type t, double x, double y, float s, float a)
 : Keypoint(x, y, s, a),
   flags(0),
   type(t),
   vec(0),
   size(0),
   pos(0),
   patch(0),
   error(0),
   cntError(0)
{
}

KeypointDescriptor::KeypointDescriptor(Type t, double x, double y, float s, float a,
                                       float _m11,float _m12,float _m21,float _m22)
 : Keypoint(x, y, s, a, _m11, _m12, _m21, _m22),
   flags(0),
   type(t),
   vec(0),
   size(0),
   pos(0),
   patch(0),
   error(0),
   cntError(0)
{
}

/**
 * vote for an object
 */
bool KeypointDescriptor::GetVoteCenter(KeypointDescriptor *current,Vector2 &v,
                                       double &delta_angle, double &delta_scale)
{
  if (type==current->type){
    delta_scale = current->Scale()/Scale();
    delta_angle = ScaleAngle_mpi_pi(-Angle() + current->Angle());
    v = current->p + Rotate(-p,delta_angle)*delta_scale;
    return true;
  }
  return false;
}


/**
 * save affine normalised patch
 */
#ifdef SAVE_PATCHES
static unsigned mcnt=0;
char fname[1024];
#endif
void KeypointDescriptor::SavePatch(IplImage *img)
{
  if (!patch) patch = cvCreateImage( cvSize(PATCH_MASK_SIZE,PATCH_MASK_SIZE), IPL_DEPTH_8U, 1 );

  float sxy=(Scale()*6/PATCH_MASK_SIZE);     //6 is a stubide constant????
  CvMat* G = cvCreateMat(2,3,CV_32FC1);

  float coso = cos(-Angle())*sxy;
  float sino = sin(-Angle())*sxy;

  G->data.fl[0] = mi11*coso-mi12*sino;
  G->data.fl[1] = mi11*sino+mi12*coso;
  G->data.fl[2] = X();
  G->data.fl[3] = mi21*coso-mi22*sino;
  G->data.fl[4] = mi21*sino+mi22*coso;
  G->data.fl[5] = Y();

  cvGetQuadrangleSubPix( img, patch, G );
 
  cvReleaseMat(&G);
  #ifdef SAVE_PATCHES
  snprintf(fname,1024,"log/patch_%04d.png",mcnt);
  cvSaveImage(fname,patch);
  mcnt++;
  #endif
}

/**
 * Affine projection of the patch to img
 */
void KeypointDescriptor::ProjectPatch(KeypointDescriptor *model,IplImage *img)
{
  if (model->patch != 0){
    float delta=-PATCH_MASK_SIZE/2.;
    float sxy=(Scale()*6/PATCH_MASK_SIZE);      //6 is a stubide constant?!?
    float coso = cos(-Angle())*sxy;
    float sino = sin(-Angle())*sxy;

    CvMat* G= cvCreateMat(2,3,CV_32FC1);

    G->data.fl[0] = mi11*coso-mi12*sino;
    G->data.fl[1] = mi11*sino+mi12*coso;
    G->data.fl[2] = delta*G->data.fl[0] + delta*G->data.fl[1] + p.x;
    G->data.fl[3] = mi21*coso-mi22*sino;
    G->data.fl[4] = mi21*sino+mi22*coso;
    G->data.fl[5] = delta*G->data.fl[3] + delta*G->data.fl[4] + p.y;

    cvWarpAffine( model->patch, img, G,CV_INTER_LINEAR);

    cvReleaseMat(&G);
  }
}

/**
 * draw a keypoint
 */
void KeypointDescriptor::Draw(IplImage *img, KeypointDescriptor &k, CvScalar col)
{
  double scale = k.Scale()*3;
  double U[4], D[4], Vi[4], V[4];
  CvMat matU = cvMat(2,2, CV_64F, U);
  CvMat matD = cvMat(2,2, CV_64F, D);
  CvMat matVi = cvMat(2,2, CV_64F, Vi);
  CvMat matV = cvMat(2,2, CV_64F, V);

  U[0] = k.Mi11(), U[1] = k.Mi12();
  U[2] = k.Mi21(), U[3] = k.Mi22();

  cvSVD(&matU, &matD, &matVi, &matV);
  
  D[0] *= scale;
  D[1] *= scale;
  D[2] *= scale;
  D[3] *= scale;

  double angle = ScaleAngle_0_2pi(acos(V[0]))*180./M_PI;
  SDraw::DrawEllipse(img, k.X(), k.Y(), D[0], D[3],angle, col);
  Vector2 v = Vector2(D[3],0);
  v = Rotate(v,-k.Angle());
  v = Vector2(v.x+k.X(), v.y+k.Y());
  SDraw::DrawLine(img, k.X(), k.Y(), v.x, v.y, col);
}

/**
 * read/write keypoints
 */
void KeypointDescriptor::ReadKeypoint( ifstream &in )
{
  float tmp;

  in >> p.x;
  in >> p.y;
  in >> tmp;
  in >> scale;
  in >> angle;
  in >> tmp;
  in >> tmp;
  in >> tmp;
  in >> tmp;
  in >> mi11;
  in >> mi12;
  in >> mi21;
  in >> mi22;
}

void KeypointDescriptor::ReadDescriptor( ifstream &in, unsigned size_in)
{
  if(size_in>0){
    AllocVec(size_in);
    for(unsigned j=0; j<size;j++){
      in >> vec[j];
    }
  }
}

void KeypointDescriptor::WriteKeypoint( ofstream &out )
{
  out << p.x << " " << p.y << " " << 0 << " " << scale << " ";
  out << angle<< " "<< type << " " << 0 << " " <<0<< " " <<0;
  out << " " << mi11 << " " << mi12 << " " << mi21 << " " << mi22<< " ";
}

void KeypointDescriptor::WriteDescriptor(ofstream &out )
{
  if(size>0){
    for(unsigned j=0; j<size;j++){
      out << vec[j] << " ";
     }
  }
}

void WriteKeypoints(P::Array<KeypointDescriptor*> &ks, const char* file, int format)
{
  if (ks.Size()==0){
      cout << " Keypoints Nb: " << ks.Size() << endl;
      return;
  }
    ofstream output(file);
    if(!output)cout << "Error opening " << file<< endl;
    output << ks[0]->GetSize()<< endl;
    output << ks.Size()<< endl;
    for(unsigned i=0; i<ks.Size();i++){
      if(format==0){
        ks[i]->WriteKeypoint(output);
        ks[i]->WriteDescriptor(output);
        output<<endl;
      }
    }
    output.close();
}

void LoadKeypoints(const char* file, P::Array<KeypointDescriptor*> &ks, int format)
{
  KeypointDescriptor* k;
  ifstream input1(file);

  if(!input1)return;

  unsigned numip, size;
  float tmp;
  input1 >> tmp;
  input1 >> numip;
  if(tmp<=1.0) size=0;
  else size=(int)tmp;
  for (unsigned i=0; i<numip; i++){
    if(format==0){
      k = new KeypointDescriptor();
      k->ReadKeypoint(input1);
      k->ReadDescriptor(input1, size);
      ks.PushBack(k);
    }
  }

  input1.close();
}

void LoadLoweKeypoints(const char* file, P::Array<Keypoint*> &ks, int format)
{
  KeypointDescriptor* k;
  ifstream input1(file);

  if(!input1)return;

  unsigned numip, size;
  float tmp;
  input1 >> numip;
  input1 >> tmp;
  if(tmp<=1.0) size=0;
  else size=(int)tmp;
  for (unsigned i=0; i<numip; i++){
    if(format==0){
      k = new KeypointDescriptor();
      input1 >> k->p.y;
      input1 >> k->p.x;
      input1 >> k->scale;
      input1 >> k->angle;
      k->angle = -k->angle;
      k->ReadDescriptor(input1, size);
      ks.PushBack(k);
    }
  }

  input1.close();
}

void LoadLoweKeypoints(const char* file, P::Array<KeypointDescriptor*> &ks, int format)
{
  KeypointDescriptor* k;
  ifstream input1(file);

  if(!input1)return;

  unsigned numip, size;
  float tmp;
  input1 >> numip;
  input1 >> tmp;
  if(tmp<=1.0) size=0;
  else size=(int)tmp;
  for (unsigned i=0; i<numip; i++){
    if(format==0){
      k = new KeypointDescriptor();
      input1 >> k->p.y;
      input1 >> k->p.x;
      input1 >> k->scale;
      input1 >> k->angle;
      k->angle = -k->angle;
      k->ReadDescriptor(input1, size);
      ks.PushBack(k);
    }
  }

  input1.close();
}


void KeypointDescriptor::SaveAll(ofstream &os, const KeypointDescriptor &k)
{
  os << k.p.x<<' '<<k.p.y<<' '<<k.scale<<' '<<k.angle;
  os <<' '<<k.mi11<<' '<<k.mi12<<' '<<k.mi21<<' '<<k.mi22;
  os <<' '<<k.flags<<' '<<k.type<<' '<<k.size;
  for (unsigned i=0; i<k.size; i++)
    os <<' '<<k.vec[i];
  if (k.pos!=0)
    os <<' '<<'1'<<' '<<(double)cvmGet(k.pos,0,0)<<' '<<(double)cvmGet(k.pos,1,0)<<' '<<(double)cvmGet(k.pos,2,0);
  else
    os <<' '<<'0';
  os<<' '<<k.error<<' '<<k.cntError;
  os<<'\n';

}

void KeypointDescriptor::LoadAll(ifstream &is, KeypointDescriptor &k)
{
  is >> k.p.x>>k.p.y>>k.scale>>k.angle>>k.mi11>>k.mi12>>k.mi21>>k.mi22;
  unsigned type;
  is >>k.flags>>type>>k.size;
  k.type = (KeypointDescriptor::Type)type;
  k.AllocVec(k.size);
  for (unsigned i=0; i<k.size; i++)
    is >>k.vec[i];
  unsigned c;
  is>>c;
  if (c==1)
  {
    if (k.pos==0) k.pos = cvCreateMat(3,1, CV_64F);
    is>>k.pos->data.db[0]>>k.pos->data.db[1]>>k.pos->data.db[2];
  }
  is>>k.error>>k.cntError;
}

void CopyKeypoints(Array<KeypointDescriptor*> &src, Array<KeypointDescriptor*> &dst)
{
  KeypointDescriptor *key;
  DeleteKeypoints(dst);

  for (unsigned i=0; i<src.Size(); i++)
  {
    key = new KeypointDescriptor(src[i]);
    key->bw = src[i];
    src[i]->fw = key;
    dst.PushBack(key);
  }
}

void DeleteKeypoints(Array<KeypointDescriptor*> &keys)
{
  KeypointDescriptor *key;
  for (unsigned i=0; i<keys.Size(); i++)
  {
    key = keys[i];
    if (key->fw!=0) key->fw->bw = 0;
    if (key->bw!=0) key->bw->fw = 0;
    delete key;
  }
  keys.Clear();
}

void DeleteKeypoint(KeypointDescriptor* key)
{
  if (key->fw!=0) key->fw->bw = 0;
  if (key->bw!=0) key->bw->fw = 0;
  delete key;
}


}












