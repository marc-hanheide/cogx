//UTILITY FOR GENERATING COLOR PARAMETERS
//(2008) Peter Rulic
//---------------------------------------------------------------------------
#include <stdio.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
//---------------------------------------------------------------------------
//constants
#define NB  10   //number of brightest pixels in image 

//global variables
long p1,p2,p3,p4; //angles of a white paper
unsigned char threshold; //threshold for white paper segmentation

struct PIKSLI               
{
 float* r;
 float* g;
 float* b;
 long st;
 } piksli, piksli_;         

float mi_min, mi_max, mi_delta;  
float fi_min, fi_max, fi_delta;  
float min_proc_premice;   

float t_min, t_max;            
float k_r, n_r;                
float k_g, n_g;                
float k_b, n_b;                
float ve_r, ve_g, ve_b;        
float a_elipse, b_elipse;     
float a_elipse_k, b_elipse_k;  
long st_seg;
//---------------------------------------------------------------------------
//writing color parameters in file
void dump(void)
{
 long i;
 FILE *izh;
 izh = fopen("color.txt","wt");
 fprintf(izh,"automatically generated file\n\n");
 fprintf(izh,"TARGET COLOR PARAMETERS:\n");
 fprintf(izh,"********************************\n");
 fprintf(izh,"t_min=%f;\nt_max=%f;\n",t_min, t_max);                   
 fprintf(izh,"k_r=%f;\nn_r=%f;\n",k_r,n_r);
 fprintf(izh,"k_g=%f;\nn_g=%f;\n",k_g,n_g);
 fprintf(izh,"k_b=%f;\nn_b=%f;\n",k_b,n_b);
 fprintf(izh,"smv_r=%f;\nsmv_g=%f;\nsmv_b=%f;\n",ve_r,ve_g,ve_b);
 fprintf(izh,"elipsa_a=%f;\nelipsa_b=%f;\n",a_elipse, b_elipse);               
 fprintf(izh,"k_elipsa_a=%f;\nk_elipsa_b=%f;\n",a_elipse_k, b_elipse_k);
 fprintf(izh,"********************************\n");
 fclose(izh);
}

//find corners of rectangular white paper
void find_angles(IplImage* slika)
{
 long hor,ver;
 long i,j;
 long d;
 unsigned char r,g,b;
 long d1,d2,d3,d4; 

 hor=slika->width;
 ver=slika->height;  
 
 p1=0;
 p2=0;
 p3=0;
 p4=0;

 //find corners of white paper
 d1=hor*ver;
 d2=hor*ver;
 d3=hor*ver;
 d4=hor*ver;
 for(i=0; i<hor*ver; i++)
 { 
  j=3*i;
  b=slika->imageData[j];
  g=slika->imageData[j+1];
  r=slika->imageData[j+2];
  if(b>=threshold)
//  if((r>=threshold) && (g>=threshold) && (b>=threshold)) 
  {
   slika->imageData[j]=0;
   slika->imageData[j+1]=0;
   slika->imageData[j+2]=0;
   d=i%hor;
   if(d<d1) {p1=i;  d1=d;} //position closest to the left image side
   d=i/hor;
   if(d<=d2) {p2=i; d2=d;} //position closest to the up image side
   d=hor-1-(i%hor);
   if(d<=d3) {p3=i; d3=d;} //position closest to the right image side
   d=ver-1-(i/hor);
   if(d<d4) {p4=i;  d4=d;} //position closest to bottom image side
  }
 } 
}

//find approximate white paper rgb values
void find_threshold(IplImage* slika)
{
 long i, j, hor, ver;
 unsigned char r,g,b;
 unsigned char t,tmax;

 hor=slika->width;
 ver=slika->height;  
 
 tmax=0;
 for(i=0; i<hor*ver; i++)
 {
  j=3*i;
  b=slika->imageData[j];
  g=slika->imageData[j+1];
  r=slika->imageData[j+2];
  
  t=b;
  if(g<t) t=g;
  if(r<t) t=r;
  
  if(t>tmax) tmax=t;
 }

 threshold=tmax-20;
}

//capture pixels of target color above white paper
void GetPixels(IplImage *slika)
{
 long hor,ver;
 long i,j,k,l,m;
 unsigned char rrrr,gggg,bbbb;
 long grp[10],sg, grps, grpe;

 hor=slika->width;
 ver=slika->height;  

 //get target color pixels
 for(i=0; i<ver; i++)
 {
  sg=0;  
  for(j=0; j<hor; j++)
  {
   l=3*((hor*i)+j); 
   bbbb=slika->imageData[l];
   gggg=slika->imageData[l+1];
   rrrr=slika->imageData[l+2];
   if((bbbb==0)&&(gggg==255)&&(rrrr==0)&&(sg<10)) 
   {
    grp[sg]=j; 
    ++sg;
   }
  }

  //find start and end pixel within green frame
  grps=0;
  grpe=0;
  if(sg>1)
  {
   m=1;
   while((grp[m]-1==grp[m-1])&&(m<sg)) ++m;
   if(m<sg)
   {
    grpe=grp[m];
    grps=grp[m-1];
   }
  }

  if(grps+20<grpe)
  {
   for(j=grps+1; j<grpe; j++)
   {
    l=3*((hor*i)+j); 
    bbbb=slika->imageData[l];
    gggg=slika->imageData[l+1];
    rrrr=slika->imageData[l+2];
    if((bbbb!=0)&&(gggg!=0)&&(rrrr!=0)) 
    { 
     piksli.r[piksli.st]=(float)rrrr;
     piksli.g[piksli.st]=(float)gggg;
     piksli.b[piksli.st]=(float)bbbb;
     ++piksli.st;
    }
   }
  }
 }
}

//generate model prarameters from captured pixels 
int GenerateModel()
{
 long hor,ver;
 long i,j,l,m;
 long skpm, skmp, skpp, skmm, kat;      
 float r,g,b;
 float fi, mi, naj_fi, naj_mi;     
 float cosfi, sinficosmi, sinfisinmi, _sinmi, cosmi, _sinfi, cosficosmi, cosfisinmi;
 float sr_vrednost_r, naj_sr_vrednost_r, sr_vrednost_g, naj_sr_vrednost_g;
 float sr_razd, naj_sr_razd, aa,bb,cc,dd;
 float a_elipse_min, b_elipse_min, a_elipse_max, b_elipse_max;  
 float sv_r_min,sv_g_min, sv_r_max,sv_g_max; 
 float ve_r_min, ve_g_min, ve_b_min;   
 float ve_r_max, ve_g_max, ve_b_max;   

 if(piksli.st==0) return -10;

 //process color pixels
 naj_sr_razd=99e30;   
 piksli_.st=piksli.st;
 
 for(fi=fi_min; fi<=fi_max; fi=fi+fi_delta)   
  for(mi=mi_min; mi<=mi_max; mi=mi+mi_delta)   
  {
   cosfi=cos(fi);
   sinficosmi=sin(fi)*cos(mi);
   sinfisinmi=sin(fi)*sin(mi);
   _sinmi=(-1)*sin(mi);
   cosmi=cos(mi);
   _sinfi=(-1)*sin(fi);
   cosficosmi=cos(fi)*cos(mi);
   cosfisinmi=cos(fi)*sin(mi);

   for(i=0; i<piksli.st; i++)
   {
    piksli_.r[i]=(piksli.r[i]*cosfi) + (piksli.g[i]*sinficosmi) + (piksli.b[i]*sinfisinmi);
    piksli_.g[i]=(piksli.g[i]*_sinmi) + (piksli.b[i]*cosmi);
    piksli_.b[i]=(piksli.r[i]*_sinfi) + (piksli.g[i]*cosficosmi) + (piksli.b[i]*cosfisinmi);
   }

   sr_vrednost_r=0;
   sr_vrednost_g=0;
   for(i=0; i<piksli_.st; i++)
   {
    sr_vrednost_r=sr_vrednost_r+piksli_.r[i];
    sr_vrednost_g=sr_vrednost_g+piksli_.g[i];
   }
   sr_vrednost_r=sr_vrednost_r/piksli_.st;
   sr_vrednost_g=sr_vrednost_g/piksli_.st;

   sr_razd=0;
   for(i=0; i<piksli_.st; i++)
   {
    sr_razd=sr_razd + fabs(piksli_.r[i]-sr_vrednost_r)+fabs(piksli_.g[i]-sr_vrednost_g);
   }
   sr_razd=sr_razd/piksli_.st;

   if(sr_razd<naj_sr_razd)  
   {
    naj_sr_razd=sr_razd;
    naj_sr_vrednost_r=sr_vrednost_r;
    naj_sr_vrednost_g=sr_vrednost_g;
    naj_fi=fi;
    naj_mi=mi;
   }
  }

  cosfi=cos(naj_fi);
  sinficosmi=sin(naj_fi)*cos(naj_mi);
  sinfisinmi=sin(naj_fi)*sin(naj_mi);
  _sinmi=(-1)*sin(naj_mi);
  cosmi=cos(naj_mi);
  _sinfi=(-1)*sin(naj_fi);
  cosficosmi=cos(naj_fi)*cos(naj_mi);
  cosfisinmi=cos(naj_fi)*sin(naj_mi);

  for(i=0; i<piksli.st; i++)
  {
   piksli_.r[i]=(piksli.r[i]*cosfi) + (piksli.g[i]*sinficosmi) + (piksli.b[i]*sinfisinmi);
   piksli_.g[i]=(piksli.g[i]*_sinmi) + (piksli.b[i]*cosmi);
   piksli_.b[i]=(piksli.r[i]*_sinfi) + (piksli.g[i]*cosficosmi) + (piksli.b[i]*cosfisinmi);
  }

 l=(piksli_.st-1)/3;   
 for(i=0; i<piksli_.st-1; i++)
  for(j=piksli_.st-1; j>i; j--)
   if(piksli_.b[j]<piksli_.b[j-1])
   {
    r=piksli_.r[j];
    g=piksli_.g[j];
    b=piksli_.b[j];

    piksli_.r[j]=piksli_.r[j-1];
    piksli_.g[j]=piksli_.g[j-1];
    piksli_.b[j]=piksli_.b[j-1];

    piksli_.r[j-1]=r;
    piksli_.g[j-1]=g;
    piksli_.b[j-1]=b;
   }

 j=(long)((float)l*min_proc_premice); 
 t_min=0;
 for(i=0; i<j; i++) t_min=t_min+piksli_.b[i];
 t_min=t_min/(float)j;

 j=piksli_.st-j;   
 t_max=0;
 for(i=piksli_.st-1; i>j; i--) t_max=t_max+piksli_.b[i];
 j=(long)((float)l*min_proc_premice);  
 t_max=t_max/(float)j;

 for(i=0; i<l; i++)
  for(j=l; j>i; j--)
  {
   aa=fabsf(naj_sr_vrednost_r-piksli_.r[j])   + fabsf(naj_sr_vrednost_g-piksli_.g[j]);
   bb=fabsf(naj_sr_vrednost_r-piksli_.r[j-1]) + fabsf(naj_sr_vrednost_g-piksli_.g[j-1]);
   if( aa>bb)
   {
    r=piksli_.r[j];
    g=piksli_.g[j];
    b=piksli_.b[j];

    piksli_.r[j]=piksli_.r[j-1];
    piksli_.g[j]=piksli_.g[j-1];
    piksli_.b[j]=piksli_.b[j-1];

    piksli_.r[j-1]=r;
    piksli_.g[j-1]=g;
    piksli_.b[j-1]=b;
   }
  }
 j=(long)((float)l*min_proc_premice);   
 a_elipse_min=0.0;
 for(i=0; i<j; i++) a_elipse_min=a_elipse_min + fabsf(piksli_.r[i]-naj_sr_vrednost_r) + fabsf(piksli_.g[i]-naj_sr_vrednost_g);
 a_elipse_min=a_elipse_min/(float)j;

 skpm=0;
 skmp=0;
 skpp=0;
 skmm=0;
 for(i=0; i<j; i++)
 {
  if((piksli_.r[i]-naj_sr_vrednost_r<0)  && (piksli_.g[i]-naj_sr_vrednost_g<0))  ++skmm; 
  if((piksli_.r[i]-naj_sr_vrednost_r<0)  && (piksli_.g[i]-naj_sr_vrednost_g>=0)) ++skmp; 
  if((piksli_.r[i]-naj_sr_vrednost_r>=0) && (piksli_.g[i]-naj_sr_vrednost_g<0))  ++skpm; 
  if((piksli_.r[i]-naj_sr_vrednost_r>=0) && (piksli_.g[i]-naj_sr_vrednost_g>=0)) ++skpp; 
 }
 if((skmm>=skmp)&&(skmm>=skpm)&&(skmm>=skpp)) kat=1;  
 if((skmp>=skmm)&&(skmp>=skpm)&&(skmp>=skpp)) kat=2;  
 if((skpm>=skmm)&&(skpm>=skmp)&&(skpm>=skpp)) kat=3;  
 if((skpp>=skmm)&&(skpp>=skmp)&&(skpp>=skpm)) kat=4;  

 sv_r_min=0;
 sv_g_min=0;
 if(kat==1)   
  for(i=0; i<j; i++)
   if( (piksli_.r[i]-naj_sr_vrednost_r<0) | ((piksli_.r[i]-naj_sr_vrednost_r>=0)&&(piksli_.g[i]-naj_sr_vrednost_g<0)) )
   {
    sv_r_min=sv_r_min+piksli_.r[i]-naj_sr_vrednost_r;
    sv_g_min=sv_g_min+piksli_.g[i]-naj_sr_vrednost_g;
   }
 if(kat==2)   
  for(i=0; i<j; i++)
   if( (piksli_.r[i]-naj_sr_vrednost_r<0) | ((piksli_.r[i]-naj_sr_vrednost_r>=0)&&(piksli_.g[i]-naj_sr_vrednost_g>=0)) )
   {
    sv_r_min=sv_r_min+piksli_.r[i]-naj_sr_vrednost_r;
    sv_g_min=sv_g_min+piksli_.g[i]-naj_sr_vrednost_g;
   }
 if(kat==3)  
  for(i=0; i<j; i++)
   if( (piksli_.g[i]-naj_sr_vrednost_g<0) | ((piksli_.r[i]-naj_sr_vrednost_r>=0)&&(piksli_.g[i]-naj_sr_vrednost_g>=0)) )
   {
    sv_r_min=sv_r_min+piksli_.r[i]-naj_sr_vrednost_r;
    sv_g_min=sv_g_min+piksli_.g[i]-naj_sr_vrednost_g;
   }
 if(kat==4)   
  for(i=0; i<j; i++)
   if( (piksli_.g[i]-naj_sr_vrednost_g>=0) | ((piksli_.r[i]-naj_sr_vrednost_r>=0)&&(piksli_.g[i]-naj_sr_vrednost_g<0)) )
   {
    sv_r_min=sv_r_min+piksli_.r[i]-naj_sr_vrednost_r;
    sv_g_min=sv_g_min+piksli_.g[i]-naj_sr_vrednost_g;
   }
 
 ve_r_min=(sv_r_min*cosfi);
 ve_g_min=(sv_r_min*sinficosmi) + (sv_g_min*_sinmi);
 ve_b_min=(sv_r_min*sinfisinmi) + (sv_g_min*cosmi);

 aa=sqrt((ve_r_min*ve_r_min)+(ve_g_min*ve_g_min)+(ve_b_min*ve_b_min));
 if(aa==0.0) aa=1e-12;  
 ve_r_min=ve_r_min/aa;
 ve_g_min=ve_g_min/aa;
 ve_b_min=ve_b_min/aa;

 aa=fabsf(sv_r_min)+fabsf(sv_g_min);
 if(aa==0.0) aa=1e-12;   
 sv_r_min=sv_r_min/aa;   
 sv_g_min=sv_g_min/aa;  
 for(i=0; i<l; i++)
  for(j=l; j>i; j--)
  {
   aa=piksli_.r[j]-naj_sr_vrednost_r;
   bb=piksli_.g[j]-naj_sr_vrednost_g;
   cc=fabsf(aa)+fabsf(bb);   
   if(cc==0.0) cc=1;   
   aa=aa/cc;  
   bb=bb/cc; 
   cc=(aa*sv_r_min)+(bb*sv_g_min);     
   aa=piksli_.r[j-1]-naj_sr_vrednost_r;
   bb=piksli_.g[j-1]-naj_sr_vrednost_g;
   dd=fabsf(aa)+fabsf(bb);
   if(dd==0.0) dd=1;  
   aa=aa/dd; 
   bb=bb/dd;  
   dd=(aa*sv_r_min)+(bb*sv_g_min); 
   if(cc<dd)
   {
    r=piksli_.r[j];
    g=piksli_.g[j];
    b=piksli_.b[j];
    piksli_.r[j]=piksli_.r[j-1];
    piksli_.g[j]=piksli_.g[j-1];
    piksli_.b[j]=piksli_.b[j-1];
    piksli_.r[j-1]=r;
    piksli_.g[j-1]=g;
    piksli_.b[j-1]=b;
   }
  }

 m=l/2;
 for(i=0; i<m; i++)
  for(j=m; j>i; j--)
  {
   aa=fabsf(naj_sr_vrednost_r-piksli_.r[j])   + fabsf(naj_sr_vrednost_g-piksli_.g[j]);
   bb=fabsf(naj_sr_vrednost_r-piksli_.r[j-1]) + fabsf(naj_sr_vrednost_g-piksli_.g[j-1]);
   if( aa>bb)
   {
    r=piksli_.r[j];
    g=piksli_.g[j];
    b=piksli_.b[j];
    piksli_.r[j]=piksli_.r[j-1];
    piksli_.g[j]=piksli_.g[j-1];
    piksli_.b[j]=piksli_.b[j-1];
    piksli_.r[j-1]=r;
    piksli_.g[j-1]=g;
    piksli_.b[j-1]=b;
   }
  }

 j=(long)((float)l*min_proc_premice);   
 cc=0.0;
 for(i=0; i<j; i++)
 {
  aa=piksli_.r[i]-naj_sr_vrednost_r;
  bb=piksli_.g[i]-naj_sr_vrednost_g;
  cc=cc+fabsf(aa)+fabsf(bb);  
 }
 if(j==0) j=1;  
 b_elipse_min=cc/(float)j;

 for(i=0; i<piksli_.st-1; i++)
  for(j=piksli_.st-1; j>i; j--)
   if(piksli_.b[j]>piksli_.b[j-1])
   {
    r=piksli_.r[j];
    g=piksli_.g[j];
    b=piksli_.b[j];

    piksli_.r[j]=piksli_.r[j-1];
    piksli_.g[j]=piksli_.g[j-1];
    piksli_.b[j]=piksli_.b[j-1];

    piksli_.r[j-1]=r;
    piksli_.g[j-1]=g;
    piksli_.b[j-1]=b;
   }

 for(i=0; i<l; i++)
  for(j=l; j>i; j--)
  {
   aa=fabsf(naj_sr_vrednost_r-piksli_.r[j])   + fabsf(naj_sr_vrednost_g-piksli_.g[j]);
   bb=fabsf(naj_sr_vrednost_r-piksli_.r[j-1]) + fabsf(naj_sr_vrednost_g-piksli_.g[j-1]);
   if( aa>bb)
   {
    r=piksli_.r[j];
    g=piksli_.g[j];
    b=piksli_.b[j];

    piksli_.r[j]=piksli_.r[j-1];
    piksli_.g[j]=piksli_.g[j-1];
    piksli_.b[j]=piksli_.b[j-1];

    piksli_.r[j-1]=r;
    piksli_.g[j-1]=g;
    piksli_.b[j-1]=b;
   }
  }

 j=(long)((float)l*min_proc_premice);   
 a_elipse_max=0.0;
 for(i=0; i<j; i++) a_elipse_max=a_elipse_max + fabsf(piksli_.r[i]-naj_sr_vrednost_r) + fabsf(piksli_.g[i]-naj_sr_vrednost_g);
 a_elipse_max=a_elipse_max/(float)j;

 skpm=0;
 skmp=0;
 skpp=0;
 skmm=0;
 for(i=0; i<j; i++)
 {
  if((piksli_.r[i]-naj_sr_vrednost_r<0)  && (piksli_.g[i]-naj_sr_vrednost_g<0))  ++skmm; 
  if((piksli_.r[i]-naj_sr_vrednost_r<0)  && (piksli_.g[i]-naj_sr_vrednost_g>=0)) ++skmp; 
  if((piksli_.r[i]-naj_sr_vrednost_r>=0) && (piksli_.g[i]-naj_sr_vrednost_g<0))  ++skpm; 
  if((piksli_.r[i]-naj_sr_vrednost_r>=0) && (piksli_.g[i]-naj_sr_vrednost_g>=0)) ++skpp; 
 }
 if((skmm>=skmp)&&(skmm>=skpm)&&(skmm>=skpp)) kat=1;  
 if((skmp>=skmm)&&(skmp>=skpm)&&(skmp>=skpp)) kat=2;  
 if((skpm>=skmm)&&(skpm>=skmp)&&(skpm>=skpp)) kat=3;  
 if((skpp>=skmm)&&(skpp>=skmp)&&(skpp>=skpm)) kat=4;  

 sv_r_max=0;
 sv_g_max=0;
 if(kat==1)   
  for(i=0; i<j; i++)
   if( (piksli_.r[i]-naj_sr_vrednost_r<0) | ((piksli_.r[i]-naj_sr_vrednost_r>=0)&&(piksli_.g[i]-naj_sr_vrednost_g<0)) )
   {
    sv_r_max=sv_r_max+piksli_.r[i]-naj_sr_vrednost_r;
    sv_g_max=sv_g_max+piksli_.g[i]-naj_sr_vrednost_g;
   }
 if(kat==2)  
  for(i=0; i<j; i++)
   if( (piksli_.r[i]-naj_sr_vrednost_r<0) | ((piksli_.r[i]-naj_sr_vrednost_r>=0)&&(piksli_.g[i]-naj_sr_vrednost_g>=0)) )
   {
    sv_r_max=sv_r_max+piksli_.r[i]-naj_sr_vrednost_r;
    sv_g_max=sv_g_max+piksli_.g[i]-naj_sr_vrednost_g;
   }
 if(kat==3) 
  for(i=0; i<j; i++)
   if( (piksli_.g[i]-naj_sr_vrednost_g<0) | ((piksli_.r[i]-naj_sr_vrednost_r>=0)&&(piksli_.g[i]-naj_sr_vrednost_g>=0)) )
   {
    sv_r_max=sv_r_max+piksli_.r[i]-naj_sr_vrednost_r;
    sv_g_max=sv_g_max+piksli_.g[i]-naj_sr_vrednost_g;
   }
 if(kat==4) 
  for(i=0; i<j; i++)
   if( (piksli_.g[i]-naj_sr_vrednost_g>=0) | ((piksli_.r[i]-naj_sr_vrednost_r>=0)&&(piksli_.g[i]-naj_sr_vrednost_g<0)) )
   {
    sv_r_max=sv_r_max+piksli_.r[i]-naj_sr_vrednost_r;
    sv_g_max=sv_g_max+piksli_.g[i]-naj_sr_vrednost_g;
   }

 ve_r_max=(sv_r_max*cosfi);
 ve_g_max=(sv_r_max*sinficosmi) + (sv_g_max*_sinmi);
 ve_b_max=(sv_r_max*sinfisinmi) + (sv_g_max*cosmi);

 aa=fabsf(ve_r_max)+fabsf(ve_g_max)+fabsf(ve_b_max);
 if(aa==0.0) aa=1e-12;   
 ve_r_max=ve_r_max/aa;
 ve_g_max=ve_g_max/aa;
 ve_b_max=ve_b_max/aa;

 aa=fabsf(sv_r_max)+fabsf(sv_g_max);
 if(aa==0.0) aa=1e-12; 
 sv_r_max=sv_r_max/aa;   
 sv_g_max=sv_g_max/aa;    
 for(i=0; i<l; i++)
  for(j=l; j>i; j--)
  {
   aa=piksli_.r[j]-naj_sr_vrednost_r;
   bb=piksli_.g[j]-naj_sr_vrednost_g;
   cc=fabsf(aa)+fabsf(bb);  
   if(cc==0.0) cc=1.0;  
   aa=aa/cc;  
   bb=bb/cc;  
   cc=(aa*sv_r_max)+(bb*sv_g_max);  
   aa=piksli_.r[j-1]-naj_sr_vrednost_r;
   bb=piksli_.g[j-1]-naj_sr_vrednost_g;
   dd=fabsf(aa)+fabsf(bb);
   if(dd==0.0) dd=1.0;  
   aa=aa/dd;  
   bb=bb/dd;  
   dd=(aa*sv_r_max)+(bb*sv_g_max);  
   if(cc<dd)
   {
    r=piksli_.r[j];
    g=piksli_.g[j];
    b=piksli_.b[j];

    piksli_.r[j]=piksli_.r[j-1];
    piksli_.g[j]=piksli_.g[j-1];
    piksli_.b[j]=piksli_.b[j-1];

    piksli_.r[j-1]=r;
    piksli_.g[j-1]=g;
    piksli_.b[j-1]=b;
   }
  }

 m=l/2;
 for(i=0; i<m; i++)
  for(j=m; j>i; j--)
  {
   aa=fabsf(naj_sr_vrednost_r-piksli_.r[j])   + fabsf(naj_sr_vrednost_g-piksli_.g[j]);
   bb=fabsf(naj_sr_vrednost_r-piksli_.r[j-1]) + fabsf(naj_sr_vrednost_g-piksli_.g[j-1]);
   if( aa>bb)
   {
    r=piksli_.r[j];
    g=piksli_.g[j];
    b=piksli_.b[j];
    piksli_.r[j]=piksli_.r[j-1];
    piksli_.g[j]=piksli_.g[j-1];
    piksli_.b[j]=piksli_.b[j-1];
    piksli_.r[j-1]=r;
    piksli_.g[j-1]=g;
    piksli_.b[j-1]=b;
   }
  }

 j=(long)((float)l*min_proc_premice);   
 cc=0;
 for(i=0; i<j; i++)
 {
  aa=piksli_.r[i]-naj_sr_vrednost_r;
  bb=piksli_.g[i]-naj_sr_vrednost_g;
  cc=cc+fabsf(aa)+fabsf(bb);  
 }
 if(j==0) j=1;  
 b_elipse_max=cc/(float)j;

 k_r=_sinfi;
 k_g=cosficosmi;
 k_b=cosfisinmi;
 n_r=(naj_sr_vrednost_r*cosfi);
 n_g=(naj_sr_vrednost_r*sinficosmi)+(naj_sr_vrednost_g*_sinmi);
 n_b=(naj_sr_vrednost_r*sinfisinmi)+(naj_sr_vrednost_g*cosmi);

 a_elipse_k=(a_elipse_max-a_elipse_min)/(t_max-t_min);
 b_elipse_k=(b_elipse_max-b_elipse_min)/(t_max-t_min);

 a_elipse=a_elipse_min;
 b_elipse=b_elipse_min;

 ve_r=ve_r_max+ve_r_min;
 ve_g=ve_g_max+ve_g_min;
 ve_b=ve_b_max+ve_b_min;
 aa=fabsf(ve_r)+fabsf(ve_g)+fabsf(ve_b);
 ve_r=ve_r/aa;
 ve_g=ve_g/aa;
 ve_b=ve_b/aa;

 if(a_elipse_max<b_elipse_max) return -1;   
 if(a_elipse_min<b_elipse_min) return -1;   

 aa=(ve_r_max*ve_r_min)+(ve_g_max*ve_g_min)+(ve_b_max*ve_b_min);
 if(aa>0.5) return -1;

 return 0; 
}

int main(int argc, char* argv[])
{
 unsigned long no_frames=0;
 int key;
 long hor,ver;
 long i; long chN;  //image width, height, number of channels
 bool tc=false;  //start capture
 CvFont font;

 IplImage* img=0;
 CvCapture* capture = cvCaptureFromCAM(0); // capture from video device #0

 //capture first frame
 if(!cvGrabFrame(capture))
 {
  printf("Could not capture image frame\n\7");
  exit(0);
 }
 img=cvRetrieveFrame(capture);           //retrieve captured frame

 //initialise the image processing
 ver=img->height;
 hor=img->width;
 chN=img->nChannels;

 //allocate mem for target color pixels 
 st_seg=0; piksli.st=0;
 piksli.r  = new float [3*hor*ver];
 piksli.g  = new float [3*hor*ver];
 piksli.b  = new float [3*hor*ver];
 piksli_.r = new float [3*hor*ver];
 piksli_.g = new float [3*hor*ver];
 piksli_.b = new float [3*hor*ver];

 //initialize the window
 cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);
 cvMoveWindow("mainWin", 100, 100);

 threshold=255;

 //PROCESSING LOOP
 if(chN==3)
 while(1)
 {
  if(no_frames==5) find_threshold(img);
  
  find_angles(img);
  //draw lines between angles
  cvLine(img, cvPoint(p1%hor,p1/hor), cvPoint(p2%hor,p2/hor), cvScalar(0,255,0), 1); //p1,p2
  cvLine(img, cvPoint(p1%hor,p1/hor), cvPoint(p4%hor,p4/hor), cvScalar(0,255,0), 1); //p1,p4
  cvLine(img, cvPoint(p3%hor,p3/hor), cvPoint(p2%hor,p2/hor), cvScalar(0,255,0), 1); //p3,p2
  cvLine(img, cvPoint(p3%hor,p3/hor), cvPoint(p4%hor,p4/hor), cvScalar(0,255,0), 1); //p3,p4

  //write text on image
  if(st_seg==0)
  {
   //write basic text data on image surface
   cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.5,0.5,0,1); //h,v scale=0.5, line width=1
   cvPutText (img,"adjust = '+' and '-'  (pres 'p' to capture 1/3 color segment)",cvPoint(10,15), &font, cvScalar(0,10,0));
  }
  //write text on image
  if(st_seg==1)
  {
   //write basic text data on image surface
   cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.5,0.5,0,1); //h,v scale=0.5, line width=1
   cvPutText (img,"adjust = '+' and '-'  (pres 'p' to capture 2/3 color segment)",cvPoint(10,15), &font, cvScalar(0,10,0));
  }
  //write text on image
  if(st_seg==2)
  {
   //write basic text data on image surface
   cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 0.5,0.5,0,1); //h,v scale=0.5, line width=1
   cvPutText (img,"adjust = '+' and '-'  (pres 'p' to capture 3/3 color segment and generate)",cvPoint(10,15), &font, cvScalar(0,10,0));
  }

  cvShowImage("mainWin",img);   //show camera image

  //get key 
  key=cvWaitKey(10);  

  //escape -> exit
  if(key==27) break;  

  //sS=> save the displayed image
  if((key=='s')||(key=='S')) 
  {
   cvSaveImage("sav_cap.bmp",img);
  } 

  //+- -> increase, decrease threshold 
  if(key=='+') ++threshold;
  if(key=='-') --threshold;
  
  //p -> process image
  if((key=='p')||(key=='P'))
  {
   ++st_seg;   
   //generate model, write the parameters and exit the program
   mi_min=0.0; 
   mi_max=90.0; 
   mi_delta=0.5;  
   fi_min=0.0; 
   fi_max=90.0;  
   fi_delta=0.5;  
   min_proc_premice=0.007;   
   GetPixels(img);
   if(st_seg==3)
   {
    printf("Processing ... \n");
    GenerateModel();
    dump();
    break;
   }
  }

  //grab new frame
  if(!cvGrabFrame(capture))
  {
   printf("Could not grab a frame\n\7");
   break;
  }
  ++no_frames;
  img=cvRetrieveFrame(capture); //retrieve captured frame

 }
 else printf("Not a color image\n\7");

 //release mem
 delete [] piksli.r;
 delete [] piksli.g;
 delete [] piksli.b;
 delete [] piksli_.r;
 delete [] piksli_.g;
 delete [] piksli_.b;
 cvDestroyWindow("mainWin");
 cvReleaseCapture(&capture);
 cvReleaseImage(&img);
 cvDestroyWindow("mainWin");
 return 0;
}
//---------------------------------------------------------------------------
