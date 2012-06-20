/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */


#include "Homography.hh"


namespace P 
{



/**
 * ComputeHomography
 * calculates the least square homography matrix (H) of two point sets (p1,p2)
 */
void ComputeHomography(Array<Vector2> &p1, Array<Vector2> &p2, Matrix &H, double *error)
{

  H.create(3,3);
  
  if(p1.Size()>=4 && p1.Size()==p2.Size()){
    CvMat *AA = cvCreateMat(2*p1.Size(), 8, CV_32FC1);
    CvMat *B  = cvCreateMat(2*p1.Size(), 1, CV_32FC1);
    CvMat *X  = cvCreateMat(8, 1, CV_32FC1);

    int row=0;
    for(unsigned i=0; i<p1.Size(); i++){
      cvmSet(AA, row, 0, 0);
      cvmSet(AA, row, 1, 0);
      cvmSet(AA, row, 2, 0);
      cvmSet(AA, row, 3, -p1[i].x);
      cvmSet(AA, row, 4, -p1[i].y);
      cvmSet(AA, row, 5, -1);
      cvmSet(AA, row, 6, p1[i].x*p2[i].y);
      cvmSet(AA, row, 7, p1[i].y*p2[i].y);
      cvmSet(B, row, 0, -p2[i].y);

      row++;

      cvmSet(AA, row, 0, p1[i].x);
      cvmSet(AA, row, 1, p1[i].y);
      cvmSet(AA, row, 2, 1);
      cvmSet(AA, row, 3, 0);
      cvmSet(AA, row, 4, 0);
      cvmSet(AA, row, 5, 0);
      cvmSet(AA, row, 6, -p1[i].x*p2[i].x);
      cvmSet(AA, row, 7, -p1[i].y*p2[i].x);

      cvmSet(B, row, 0, p2[i].x);
      row++;
    }

    int ok = cvSolve(AA, B, X, CV_SVD);
    if (ok != 1) throw Except (__HERE__,"homography estimate failure");

    H.tabMat[1][1]=cvmGet(X, 0, 0);
    H.tabMat[1][2]=cvmGet(X, 1, 0);
    H.tabMat[1][3]=cvmGet(X, 2, 0);
    H.tabMat[2][1]=cvmGet(X, 3, 0);
    H.tabMat[2][2]=cvmGet(X, 4, 0);
    H.tabMat[2][3]=cvmGet(X, 5, 0);
    H.tabMat[3][1]=cvmGet(X, 6, 0);
    H.tabMat[3][2]=cvmGet(X, 7, 0);
    H.tabMat[3][3]=1.;

    if (error!=0){
      CvMat *E = cvCreateMat(2*p1.Size(), 1, CV_32FC1);
      cvScale(B,B,-1);
      cvMatMulAdd(AA,X,B,E);
      *error=sqrt(2*Sqr(cvNorm(E,0,CV_L2))/(2*p1.Size()-8));
      cvReleaseMat(&E);
    }


    cvReleaseMat(&AA);
    cvReleaseMat(&B);
    cvReleaseMat(&X);

  }else throw Except(__HERE__,"Too view points for homography!");
}

void HomographyError(Array<Vector2> &p1, Array<Vector2> &p2, Matrix &H, double &error)
{
  if(p1.Size()>=4 && p1.Size()==p2.Size() && H.nbRows()==3 && H.nbCols()==3){
    CvMat *AA = cvCreateMat(2*p1.Size(), 8, CV_32FC1);
    CvMat *B  = cvCreateMat(2*p1.Size(), 1, CV_32FC1);
    CvMat *X  = cvCreateMat(8, 1, CV_32FC1);

    int row=0;
    for(unsigned i=0; i<p1.Size(); i++){
      cvmSet(AA, row, 0, 0);
      cvmSet(AA, row, 1, 0);
      cvmSet(AA, row, 2, 0);
      cvmSet(AA, row, 3, -p1[i].x);
      cvmSet(AA, row, 4, -p1[i].y);
      cvmSet(AA, row, 5, -1);
      cvmSet(AA, row, 6, p1[i].x*p2[i].y);
      cvmSet(AA, row, 7, p1[i].y*p2[i].y);
      cvmSet(B, row, 0, -p2[i].y);

      row++;

      cvmSet(AA, row, 0, p1[i].x);
      cvmSet(AA, row, 1, p1[i].y);
      cvmSet(AA, row, 2, 1);
      cvmSet(AA, row, 3, 0);
      cvmSet(AA, row, 4, 0);
      cvmSet(AA, row, 5, 0);
      cvmSet(AA, row, 6, -p1[i].x*p2[i].x);
      cvmSet(AA, row, 7, -p1[i].y*p2[i].x);

      cvmSet(B, row, 0, p2[i].x);
      row++;
    }

    cvmSet(X, 0, 0,H.tabMat[1][1]);
    cvmSet(X, 1, 0,H.tabMat[1][2]);
    cvmSet(X, 2, 0,H.tabMat[1][3]);
    cvmSet(X, 3, 0,H.tabMat[2][1]);
    cvmSet(X, 4, 0,H.tabMat[2][2]);
    cvmSet(X, 5, 0,H.tabMat[2][3]);
    cvmSet(X, 6, 0,H.tabMat[3][1]);
    cvmSet(X, 7, 0,H.tabMat[3][2]);

    CvMat *E = cvCreateMat(2*p1.Size(), 1, CV_32FC1);
    cvScale(B,B,-1);
    cvMatMulAdd(AA,X,B,E);
    error=sqrt(2*Sqr(cvNorm(E,0,CV_L2))/(2*p1.Size()-8));

    cvReleaseMat(&E);
    cvReleaseMat(&AA);
    cvReleaseMat(&B);
    cvReleaseMat(&X);

  }else throw Except(__HERE__,"Error!");
}

/**
 * AffineMatrix
 * calculates the affine parameter of mapped points (least square).
 * (Ax=b) => x=[At A]^-1 At b.
 * IN: cs1    ... corners frame 1
 *     cs2    ... corners frame 2
 * OUT: aff   ... 3x3 affine transformation matrix from frame 1 to fame 2
 *      error ... remaining error
 */
bool AffineMatrix(Array<Vector2> &cs1, Array<Vector2> &cs2,
                  Matrix &aff, double &error)
{
  //init
  aff.clear();

  unsigned idx;

  if (cs1.Size()==cs2.Size() && cs1.Size()>=3){
    Matrix A(cs1.Size()*2,6,0.);
    Matrix b(cs1.Size()*2,1);
    Matrix At,C;
    Matrix x;
    for (unsigned i=0; i<cs1.Size(); i++){
        idx=i*2+1;
        A(idx,1)=cs1[i].x;
        A(idx,2)=cs1[i].y;
        A(idx,5)=1.;
        A(idx+1,3)=cs1[i].x;
        A(idx+1,4)=cs1[i].y;
        A(idx+1,6)=1.;
        b(idx,1)=cs2[i].x;
        b(idx+1,1)=cs2[i].y;
    }
    //solve transformation parameter
    At=A.transpose();
    C=At*A;
    C=C.inverse();
    C=C*At;
    x=C*b;

    Matrix E;
    E=A*x-b;
    error=sqrt(2*E.norm2Sqr()/(2*cs1.Size()-6));

    //response
    if (x.isSize(6,1)){
      aff.create(3,3);
      aff(1,1)=x(1,1);     //rotation, scale
      aff(1,2)=x(2,1);
      aff(2,1)=x(3,1);
      aff(2,2)=x(4,1);
      aff(1,3)=x(5,1);     //translation
      aff(2,3)=x(6,1);
      aff(3,1)=0.; aff(3,2)=0.; aff(3,3)=1.;
      return true;
    }
  }
  return false;
}

/**
 * AffineMatrix
 * calculates the affine parameter of mapped points (least square).
 * IN: cs1    ... corners frame 1
 *     cs2    ... corners frame 2
 * OUT: aff   ... 3x3 affine transformation matrix from frame 1 to fame 2
 */
void AffineMatrix2(Array<Vector2> &cs1, Array<Vector2> &cs2, Matrix &aff)
{
  //init
  aff.clear();
  aff.create(3,3);
  double S;

  if (cs1.Size()==cs2.Size() && cs1.Size()>=3){
    S=cs1[0].x*(cs1[2].y-cs1[1].y) + cs1[1].x*(cs1[0].y-cs1[2].y) +
      cs1[2].x*(cs1[1].y-cs1[0].y);

    if (S!=0){
      S=1/S;
      //calculation of the affine paramete1r
      aff(1,1) = S * (cs1[0].y*(cs2[1].x-cs2[2].x) +
      cs1[1].y*(cs2[2].x-cs2[0].x)+ cs1[2].y*(cs2[0].x-cs2[1].x));
      aff(1,2) = S * (cs1[0].x*(cs2[2].x-cs2[1].x) +
        cs1[1].x*(cs2[0].x-cs2[2].x)+ cs1[2].x*(cs2[1].x-cs2[0].x));
      aff(2,1) = S * (cs1[0].y*(cs2[1].y-cs2[2].y) +
        cs1[1].y*(cs2[2].y-cs2[0].y)+ cs1[2].y*(cs2[0].y-cs2[1].y));
      aff(2,2) = S * (cs1[0].x*(cs2[2].y-cs2[1].y) +
        cs1[1].x*(cs2[0].y-cs2[2].y)+ cs1[2].x*(cs2[1].y-cs2[0].y));
      aff(1,3) = S* (cs1[0].x*(cs1[2].y*cs2[1].x-cs1[1].y*cs2[2].x)+
        cs1[1].x*(cs1[0].y*cs2[2].x-cs1[2].y*cs2[0].x)+
        cs1[2].x*(cs1[1].y*cs2[0].x-cs1[0].y*cs2[1].x));
      aff(2,3) = S* (cs1[0].x*(cs1[2].y*cs2[1].y-cs1[1].y*cs2[2].y)+
        cs1[1].x*(cs1[0].y*cs2[2].y-cs1[2].y*cs2[0].y)+
        cs1[2].x*(cs1[1].y*cs2[0].y-cs1[0].y*cs2[1].y));
      aff(3,1) = 0;
      aff(3,2) = 0;
      aff(3,3) = 1;
    }else {
      aff.clear();
    }
  }else 
  {
    //cout << "AffineMatrix: No valid affine transformation matrix found!\n";
  }
}

/**
 * SimilarityMatrix
 * calculates the rotation, translation and scaling parameter of mapped points (least square).
 * (Ax=b) => x=[At A]^-1 At b.
 * IN: cs1    ... corners frame 1
 *     cs2    ... corners frame 2
 * OUT: aff   ... 3x3 affine transformation matrix from frame 1 to fame 2
 *      error ... remaining error
 */
void ComputeSimilarity(Array<Vector2> &cs1, Array<Vector2> &cs2,
                       Matrix &H, double *error)
{
  //init
  H.clear();

  unsigned idx;

  if (cs1.Size()==cs2.Size() && cs1.Size()>=2){
    Matrix A(cs1.Size()*2,4);
    Matrix b(cs1.Size()*2,1);
    Matrix At,C;
    Matrix x;
    for (unsigned i=0; i<cs1.Size(); i++){
        idx=i*2+1;
        A(idx,1)=cs1[i].x;
        A(idx,2)=-cs1[i].y;
        A(idx,3)=1.;
        A(idx,4)=0.;
        A(idx+1,1)=cs1[i].y;
        A(idx+1,2)=cs1[i].x;
        A(idx+1,3)=0.;
        A(idx+1,4)=1.;
        b(idx,1)=cs2[i].x;
        b(idx+1,1)=cs2[i].y;
    }
    //solve transformation parameter
    At=A.transpose();
    C=At*A;
    C=C.inverse();
    C=C*At;
    x=C*b;
    
    if (error!=0){
      Matrix E;
      E=A*x-b;
      *error=sqrt(2*E.norm2Sqr()/(2*cs1.Size()-4));
    }

    //response
    if (x.isSize(4,1)){
      H.create(3,3);
      H(1,1)=x(1,1);     //rotation, scale
      H(1,2)=-x(2,1);
      H(2,1)=x(2,1);
      H(2,2)=x(1,1);
      H(1,3)=x(3,1);     //translation
      H(2,3)=x(4,1);
      H(3,1)=0.; H(3,2)=0.; H(3,3)=1.;
    }
  }else cout << "ComputeSimilarity: No valid transformation found!\n";
}

}

