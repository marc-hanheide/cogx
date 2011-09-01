/**
 * $Id$
 */


#include "LogPolar.hh"


namespace P 
{


/********************** LogPolar ************************
 * Constructor/Destructor
 */
LogPolar::LogPolar()
 : mapx(0), mapy(0)
{
}

LogPolar::~LogPolar()
{
  if (mapx==0) cvReleaseMat(&mapx);
  if (mapy==0) cvReleaseMat(&mapy);
}







/*********************** PUBLIC ******************************************/
/**
 * Create the arrays for mapping
 * TODO: 
 * - test map size
 */
void LogPolar::ComputeMaps(CvSize ssize, CvSize dsize, CvPoint2D32f center, double M, int flags)
{
  double* exp_tab = 0;
  float* buf = 0;

  if( M <= 0 )
  {
    printf("M should be >0\n" );
    return;
  }

  if(mapx==0) mapx = cvCreateMat( dsize.height, dsize.width, CV_32F );
  if(mapy==0) mapy = cvCreateMat( dsize.height, dsize.width, CV_32F );  

  if( !(flags & CV_WARP_INVERSE_MAP) )
  {
    int phi, rho;

    exp_tab = (double*)cvAlloc( dsize.width*sizeof(exp_tab[0])) ;

    for( rho = 0; rho < dsize.width; rho++ )
        exp_tab[rho] = std::exp(rho/M);

    for( phi = 0; phi < dsize.height; phi++ )
    {
        double cp = cos(phi*2*CV_PI/(dsize.height-1));
        double sp = sin(phi*2*CV_PI/(dsize.height-1));
        float* mx = (float*)(mapx->data.ptr + phi*mapx->step);
        float* my = (float*)(mapy->data.ptr + phi*mapy->step);

        for( rho = 0; rho < dsize.width; rho++ )
        {
            double r = exp_tab[rho];
            double x = r*cp + center.x;
            double y = r*sp + center.y;

            mx[rho] = (float)x;
            my[rho] = (float)y;
        }
    }
  }
  else
  {
      int x, y;
      CvMat bufx, bufy, bufp, bufa;
      double ascale = (ssize.height-1)/(2*CV_PI);

      buf = (float*)cvAlloc( 4*dsize.width*sizeof(buf[0]) );

      bufx = cvMat( 1, dsize.width, CV_32F, buf );
      bufy = cvMat( 1, dsize.width, CV_32F, buf + dsize.width );
      bufp = cvMat( 1, dsize.width, CV_32F, buf + dsize.width*2 );
      bufa = cvMat( 1, dsize.width, CV_32F, buf + dsize.width*3 );

      for( x = 0; x < dsize.width; x++ )
          bufx.data.fl[x] = (float)x - center.x;

      for( y = 0; y < dsize.height; y++ )
      {
          float* mx = (float*)(mapx->data.ptr + y*mapx->step);
          float* my = (float*)(mapy->data.ptr + y*mapy->step);

          for( x = 0; x < dsize.width; x++ )
              bufy.data.fl[x] = (float)y - center.y;
/**/
          cvCartToPolar( &bufx, &bufy, &bufp, &bufa );

          for( x = 0; x < dsize.width; x++ )
              bufp.data.fl[x] += 1.f;

          cvLog( &bufp, &bufp );

          for( x = 0; x < dsize.width; x++ )
          {
              double rho = bufp.data.fl[x]*M;
              double phi = bufa.data.fl[x]*ascale;

              mx[x] = (float)rho;
              my[x] = (float)phi;
          }
/**/
/*
          for( x = 0; x < dsize.width; x++ )
          {
              double xx = bufx.data.fl[x];
              double yy = bufy.data.fl[x];

              double p = log(sqrt(xx*xx + yy*yy) + 1.)*M;
              double a = atan2(yy,xx);
              if( a < 0 )
                  a = 2*CV_PI + a;
              a *= ascale;

              mx[x] = (float)p;
              my[x] = (float)a;
          }
*/
      }
  }

  cvFree( &exp_tab );
  cvFree( &buf );
}

/**
 * map image to log polar space using pre-calculated maps
 */
void LogPolar::MapImage(IplImage *src, IplImage *dst, int flags, int bgcol)
{
  if (mapx==0)
  {
    cout<<"First you have to compue the maps!"<<endl;
    return;
  }

  cvRemap( src, dst, mapx, mapy, flags, cvScalarAll(bgcol) );
}



}

