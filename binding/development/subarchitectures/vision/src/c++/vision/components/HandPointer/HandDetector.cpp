#include "HandDetector.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
/**********************************************************************************************************************************/
void HandDetector::LC_(char *fname)
{
 long i,j;
 char c=0, num[256];
 float nn[15];
 FILE *vh;
 vh=fopen(fname,"rt");
 for(i=0; i<15; i++)
 {
  while((c!='-')&&(c!='+')&&!((c>='0')&&(c<='9'))&&(c!=EOF)) c=fgetc(vh);
  j=0;
  while((c!=';')&&(c!=EOF)) { num[j]=c; c = fgetc(vh); ++j; }
  num[j]=NULL;
  nn[i]=atof(num);
 }
 fclose(vh);
 color.t_min=nn[0];
 color.t_max=nn[1];
 color.k_r=nn[2];
 color.n_r=nn[3];
 color.k_g=nn[4];
 color.n_g=nn[5];
 color.k_b=nn[6];
 color.n_b=nn[7];
 color.smv_r=nn[8];
 color.smv_g=nn[9];
 color.smv_b=nn[10];
 color.elipsa_a=nn[11];
 color.elipsa_b=nn[12];
 color.k_elipsa_a=nn[13];
 color.k_elipsa_b=nn[14];
}
/**********************************************************************************************************************************/
void HandDetector::LM_(void)   
{
 float pi,b,c;
 long i,j;
 e_mod.st=102;
 e_mod.povp=0.464199;
 e_mod.pix= new unsigned char [e_mod.st];
 e_mod.p_hor= new char [e_mod.st];
 e_mod.p_ver= new char [e_mod.st];
 i=0;
 e_mod.p_hor[i]=-25; 	e_mod.p_ver[i]=-103; ++i;	
 e_mod.p_hor[i]=-22; 	e_mod.p_ver[i]=-99;  ++i; 	
 e_mod.p_hor[i]=-19; 	e_mod.p_ver[i]=-96;  ++i;	
 e_mod.p_hor[i]=-17; 	e_mod.p_ver[i]=-93;  ++i;	
 e_mod.p_hor[i]=-15; 	e_mod.p_ver[i]=-90;  ++i;	
 e_mod.p_hor[i]=-14; 	e_mod.p_ver[i]=-86;  ++i;	
 e_mod.p_hor[i]=-14; 	e_mod.p_ver[i]=-83;  ++i;	
 e_mod.p_hor[i]=-12; 	e_mod.p_ver[i]=-80;  ++i;	
 e_mod.p_hor[i]=-11; 	e_mod.p_ver[i]=-77;  ++i;	
 e_mod.p_hor[i]=-9; 	e_mod.p_ver[i]=-73;  ++i;	
 e_mod.p_hor[i]=-7; 	e_mod.p_ver[i]=-70;  ++i;	
 e_mod.p_hor[i]=-6; 	e_mod.p_ver[i]=-67;  ++i;	
 e_mod.p_hor[i]=-4; 	e_mod.p_ver[i]=-63;  ++i;	
 e_mod.p_hor[i]=-2; 	e_mod.p_ver[i]=-60;  ++i;	
 e_mod.p_hor[i]=-1; 	e_mod.p_ver[i]=-57;  ++i;	
 e_mod.p_hor[i]=0; 	e_mod.p_ver[i]=-54;  ++i;	
 e_mod.p_hor[i]=2; 	e_mod.p_ver[i]=-50;  ++i;	
 e_mod.p_hor[i]=4; 	e_mod.p_ver[i]=-45;  ++i;	
 e_mod.p_hor[i]=7; 	e_mod.p_ver[i]=-41;  ++i;	
 e_mod.p_hor[i]=9; 	e_mod.p_ver[i]=-37;  ++i;	
 e_mod.p_hor[i]=10; 	e_mod.p_ver[i]=-34;  ++i;	
 e_mod.p_hor[i]=-32; 	e_mod.p_ver[i]=-32;  ++i;	
 e_mod.p_hor[i]=-25; 	e_mod.p_ver[i]=-32;  ++i;	
 e_mod.p_hor[i]=-38; 	e_mod.p_ver[i]=-31;  ++i;	
 e_mod.p_hor[i]=12; 	e_mod.p_ver[i]=-31;  ++i;	
 e_mod.p_hor[i]=-51; 	e_mod.p_ver[i]=-29;  ++i;	
 e_mod.p_hor[i]=-45; 	e_mod.p_ver[i]=-29;  ++i;	
 e_mod.p_hor[i]=-58; 	e_mod.p_ver[i]=-27;  ++i;	
 e_mod.p_hor[i]=-17; 	e_mod.p_ver[i]=-27;  ++i;	
 e_mod.p_hor[i]=13; 	e_mod.p_ver[i]=-27;  ++i;	
 e_mod.p_hor[i]=-63; 	e_mod.p_ver[i]=-24;  ++i;	
 e_mod.p_hor[i]=-15; 	e_mod.p_ver[i]=-24;  ++i;	
 e_mod.p_hor[i]=15; 	e_mod.p_ver[i]=-24;  ++i;	
 e_mod.p_hor[i]=-74; 	e_mod.p_ver[i]=-23;  ++i;	
 e_mod.p_hor[i]=-68; 	e_mod.p_ver[i]=-23;  ++i;	
 e_mod.p_hor[i]=-87; 	e_mod.p_ver[i]=-21;  ++i;	
 e_mod.p_hor[i]=-81; 	e_mod.p_ver[i]=-21;  ++i;	
 e_mod.p_hor[i]=-14; 	e_mod.p_ver[i]=-21;  ++i;	
 e_mod.p_hor[i]=-109; 	e_mod.p_ver[i]=-19;  ++i;	
 e_mod.p_hor[i]=-102; 	e_mod.p_ver[i]=-19;  ++i;	
 e_mod.p_hor[i]=-96; 	e_mod.p_ver[i]=-19;  ++i;	
 e_mod.p_hor[i]=17; 	e_mod.p_ver[i]=-19;  ++i;	
 e_mod.p_hor[i]=-12; 	e_mod.p_ver[i]=-18;  ++i;	
 e_mod.p_hor[i]=-9; 	e_mod.p_ver[i]=-14;  ++i;	
 e_mod.p_hor[i]=20; 	e_mod.p_ver[i]=-14;  ++i;	
 e_mod.p_hor[i]=-7; 	e_mod.p_ver[i]=-11;  ++i;	
 e_mod.p_hor[i]=-6; 	e_mod.p_ver[i]=-8;   ++i;	
 e_mod.p_hor[i]=22; 	e_mod.p_ver[i]=-8;   ++i;	
 e_mod.p_hor[i]=-4; 	e_mod.p_ver[i]=-5;   ++i;	
 e_mod.p_hor[i]=23; 	e_mod.p_ver[i]=-5;   ++i;	
 e_mod.p_hor[i]=-2; 	e_mod.p_ver[i]=-1;   ++i;	
 e_mod.p_hor[i]=25; 	e_mod.p_ver[i]=-1;   ++i;	
 e_mod.p_hor[i]=0; 	e_mod.p_ver[i]=1;    ++i;	
 e_mod.p_hor[i]=27; 	e_mod.p_ver[i]=1;    ++i;	
 e_mod.p_hor[i]=2; 	e_mod.p_ver[i]=5;    ++i;	
 e_mod.p_hor[i]=28; 	e_mod.p_ver[i]=5;    ++i;	
 e_mod.p_hor[i]=4; 	e_mod.p_ver[i]=8;    ++i;	
 e_mod.p_hor[i]=30; 	e_mod.p_ver[i]=8;    ++i;	
 e_mod.p_hor[i]=5; 	e_mod.p_ver[i]=11;   ++i;	
 e_mod.p_hor[i]=31; 	e_mod.p_ver[i]=11;   ++i;	
 e_mod.p_hor[i]=5; 	e_mod.p_ver[i]=15;   ++i;	
 e_mod.p_hor[i]=35; 	e_mod.p_ver[i]=16;   ++i;	
 e_mod.p_hor[i]=7; 	e_mod.p_ver[i]=18;   ++i;	
 e_mod.p_hor[i]=36; 	e_mod.p_ver[i]=19;   ++i;	
 e_mod.p_hor[i]=9; 	e_mod.p_ver[i]=21;   ++i;	
 e_mod.p_hor[i]=38; 	e_mod.p_ver[i]=23;   ++i;	
 e_mod.p_hor[i]=10; 	e_mod.p_ver[i]=24;   ++i;	
 e_mod.p_hor[i]=12; 	e_mod.p_ver[i]=28;   ++i;	
 e_mod.p_hor[i]=40; 	e_mod.p_ver[i]=29;   ++i;	
 e_mod.p_hor[i]=13; 	e_mod.p_ver[i]=31;   ++i;	
 e_mod.p_hor[i]=41; 	e_mod.p_ver[i]=33;   ++i;	
 e_mod.p_hor[i]=15; 	e_mod.p_ver[i]=34;   ++i;	
 e_mod.p_hor[i]=43; 	e_mod.p_ver[i]=36;   ++i;	
 e_mod.p_hor[i]=17; 	e_mod.p_ver[i]=37;   ++i;	
 e_mod.p_hor[i]=45; 	e_mod.p_ver[i]=39;   ++i;	
 e_mod.p_hor[i]=18; 	e_mod.p_ver[i]=41;   ++i;	
 e_mod.p_hor[i]=46; 	e_mod.p_ver[i]=42;   ++i;	
 e_mod.p_hor[i]=20; 	e_mod.p_ver[i]=44;   ++i;	
 e_mod.p_hor[i]=22; 	e_mod.p_ver[i]=47;   ++i;	
 e_mod.p_hor[i]=48; 	e_mod.p_ver[i]=49;   ++i;	
 e_mod.p_hor[i]=48; 	e_mod.p_ver[i]=52;   ++i;	
 e_mod.p_hor[i]=23; 	e_mod.p_ver[i]=54;   ++i;	
 e_mod.p_hor[i]=49; 	e_mod.p_ver[i]=55;   ++i;	
 e_mod.p_hor[i]=25; 	e_mod.p_ver[i]=57;   ++i;	
 e_mod.p_hor[i]=51; 	e_mod.p_ver[i]=59;   ++i;	
 e_mod.p_hor[i]=53; 	e_mod.p_ver[i]=62;   ++i;	
 e_mod.p_hor[i]=27; 	e_mod.p_ver[i]=64;   ++i;	
 e_mod.p_hor[i]=54; 	e_mod.p_ver[i]=65;   ++i;	
 e_mod.p_hor[i]=28; 	e_mod.p_ver[i]=67;   ++i;	
 e_mod.p_hor[i]=54; 	e_mod.p_ver[i]=69;   ++i;	
 e_mod.p_hor[i]=30; 	e_mod.p_ver[i]=72;   ++i;	
 e_mod.p_hor[i]=56; 	e_mod.p_ver[i]=72;   ++i;	
 e_mod.p_hor[i]=31; 	e_mod.p_ver[i]=75;   ++i;	
 e_mod.p_hor[i]=56; 	e_mod.p_ver[i]=75;   ++i;	
 e_mod.p_hor[i]=58; 	e_mod.p_ver[i]=78;   ++i;	
 e_mod.p_hor[i]=35; 	e_mod.p_ver[i]=80;   ++i;	
 e_mod.p_hor[i]=58; 	e_mod.p_ver[i]=82;   ++i;	
 e_mod.p_hor[i]=38; 	e_mod.p_ver[i]=83;   ++i;	
 e_mod.p_hor[i]=43; 	e_mod.p_ver[i]=85;   ++i;	
 e_mod.p_hor[i]=56; 	e_mod.p_ver[i]=85;   ++i;	
 e_mod.p_hor[i]=49; 	e_mod.p_ver[i]=87;   ++i;	
 e_mod.p_hor[i]=53; 	e_mod.p_ver[i]=87; 	
 for(i=0; i<e_mod.st; i++) e_mod.pix[i]=255; 
 e_mod.sin0_PI_2 = new long [1000];  
 pi=M_PI/500;
 for(i=0; i<1000; i++)
 {
  c=(float)i;
  b=256*sin(pi*c);
  j=(long)b;
  c=(float)j;
  c=b-c;
  if(c>=0.5) ++j;
  e_mod.sin0_PI_2[i]=j;
 }
}
/**********************************************************************************************************************************/
void HandDetector::CS_(unsigned char* input_image, unsigned char* output_image) 
{
 long i, j;
 float r,g,b;  
 float r_,g_,b_;  
 float aa, bb, cc, skal_prod, t, norm;
 float trenutna_elipsa_a, trenutna_elipsa_b;

 aa=(color.k_r*color.n_r) + (color.k_g*color.n_g) + (color.k_b*color.n_b);
 bb=(color.k_r*color.k_r) + (color.k_g*color.k_g) + (color.k_b*color.k_b);
 for(i=0; i<hor*ver; i++)
 {
  j=3*i;
  b=(float)input_image[j];
  g=(float)input_image[j+1];
  r=(float)input_image[j+2];
  t=( (color.k_r*r) + (color.k_g*g) + (color.k_b*b) - aa ) / bb;
  output_image[i]=0;
  if((t>=color.t_min)&&(t<=color.t_max))
  {
   trenutna_elipsa_a=color.elipsa_a+((t-color.t_min)*color.k_elipsa_a);
   trenutna_elipsa_b=color.elipsa_b+((t-color.t_min)*color.k_elipsa_b);
   cc=1.0-(trenutna_elipsa_b/trenutna_elipsa_a);
   r_=(color.k_r*t) + color.n_r;
   g_=(color.k_g*t) + color.n_g;
   b_=(color.k_b*t) + color.n_b;
   r=r-r_;
   g=g-g_;
   b=b-b_;
   skal_prod=(r*color.smv_r) + (g*color.smv_g) + (b*color.smv_b);   
   r=r-(skal_prod*cc*color.smv_r);
   g=g-(skal_prod*cc*color.smv_g);
   b=b-(skal_prod*cc*color.smv_b);
   norm = fabs(r) + fabs(g) + fabs(b);
   if(norm<=trenutna_elipsa_b) output_image[i]=255;
  }
 }
}
/**********************************************************************************************************************************/
void HandDetector::ED_PB(void) 
{
 long i,x,y;
 long hhh, vvv;             
 long st_t_pix,st_m_pix;    
 long ttx, tty;             
 unsigned char p, inc;      
 
 hhh=0;
 vvv=0;
 ttx=0;
 tty=0;
 st_t_pix=0;
 st_m_pix=0;
 for(i=0; i<par_ed.okno_vel; i++)
 {
  if(par_ed.okno[i]>0)      
  {
   ++st_t_pix;              
   p=par_ed.index_okno[i];
   y=i/par_ed.okno_sir;
   x=i-(par_ed.okno_sir*y);
   inc=0;
   if(p==0)
   {
    if(par_ed.okno[i+1]==0)        { ++vvv; inc=1;}   
    if(par_ed.okno[i-1]==0)        { --vvv; inc=1;}
    if(par_ed.okno[i-par_ed.okno_sir]==0) { --hhh; inc=1;}
    if(par_ed.okno[i+par_ed.okno_sir]==0) { ++hhh; inc=1;}
    if(inc==1) {ttx=ttx+x; tty=tty+y; ++st_m_pix;}
   }
   if(p==1)
   {
    if(par_ed.okno[i+1]==0)        { ++vvv; inc=1;}   
    if(par_ed.okno[i-1]==0)        { --vvv; inc=1;}
    if(par_ed.okno[i+par_ed.okno_sir]==0) { ++hhh; inc=1;}
    if(inc==1) {ttx=ttx+x; tty=tty+y; ++st_m_pix;}
   }
   if(p==2)
   {
    if(par_ed.okno[i+1]==0)        { ++vvv; inc=1;}   
    if(par_ed.okno[i-1]==0)        { --vvv; inc=1;}
    if(par_ed.okno[i-par_ed.okno_sir]==0) { --hhh; inc=1;}
    if(inc==1) {ttx=ttx+x; tty=tty+y; ++st_m_pix;}
   }
   if(p==3)
   {
    if(par_ed.okno[i+1]==0)        { ++vvv; inc=1;}   
    if(par_ed.okno[i-par_ed.okno_sir]==0) { --hhh; inc=1;}
    if(par_ed.okno[i+par_ed.okno_sir]==0) { ++hhh; inc=1;}
    if(inc==1) {ttx=ttx+x; tty=tty+y; ++st_m_pix;}
   }
   if(p==4)
   {
    if(par_ed.okno[i-1]==0)        { --vvv; inc=1;}   
    if(par_ed.okno[i-par_ed.okno_sir]==0) { --hhh; inc=1;}
    if(par_ed.okno[i+par_ed.okno_sir]==0) { ++hhh; inc=1;}
    if(inc==1) {ttx=ttx+x; tty=tty+y; ++st_m_pix;}
   }
   if(p==5)
   {
    if(par_ed.okno[i+1]==0)        { ++vvv; inc=1;}   
    if(par_ed.okno[i+par_ed.okno_sir]==0) { ++hhh; inc=1;}
    if(inc==1) {ttx=ttx+x; tty=tty+y; ++st_m_pix;}
   }
   if(p==6)
   {
    if(par_ed.okno[i-1]==0)        { --vvv; inc=1;}   
    if(par_ed.okno[i+par_ed.okno_sir]==0) { ++hhh; inc=1;}
    if(inc==1) {ttx=ttx+x; tty=tty+y; ++st_m_pix;}
   }
   if(p==7)
   {
    if(par_ed.okno[i+1]==0)        { ++vvv; inc=1;}   
    if(par_ed.okno[i-par_ed.okno_sir]==0) { --hhh; inc=1;}
    if(inc==1) {ttx=ttx+x; tty=tty+y; ++st_m_pix;}
   }
   if(p==8)
   {
    if(par_ed.okno[i-1]==0)        { --vvv; inc=1;} 
    if(par_ed.okno[i-par_ed.okno_sir]==0) { --hhh; inc=1;}
    if(inc==1) {ttx=ttx+x; tty=tty+y; ++st_m_pix;}
   }
  
  }
 }
 okna.naklx[okna.st]=hhh;
 okna.nakly[okna.st]=vvv;
 if(st_m_pix>=1)    
 {
  okna.tockx[okna.st]=ttx/st_m_pix;
  okna.tocky[okna.st]=tty/st_m_pix;
  if((ttx%st_m_pix)>(st_m_pix>>1)) ++okna.tockx[okna.st];
  if((tty%st_m_pix)>(st_m_pix>>1)) ++okna.tocky[okna.st];
 }
 okna.st_pp[okna.st]=st_t_pix;
 ++okna.st;
}
/**********************************************************************************************************************************/
void HandDetector::ED_DE(long zap) 
{
 long i,j,n,st;
 long h,v;
 long p;
 long strtx, strty;     
 long smerx, smery;     
 long vson;             
 float a, k, x0, y0;    
 char prvi;            
 
 for(i=0; i<par_ed.okno_vel; i++) par_ed.okno[i]=0;
 if((okna.st_pp[zap]>=par_ed.okno_vel/4) && (okna.st_pp[zap]<=(par_ed.okno_vel*3)/4))
 {
  vson=labs(okna.naklx[zap]) + labs(okna.nakly[zap]);
  if(vson>=par_ed.okno_sir)
  {
   if(okna.naklx[zap] > 0) v=-par_ed.st_hor; 
   if(okna.naklx[zap] < 0) v=par_ed.st_hor;  
   if(okna.naklx[zap]== 0) v=0;              
   if(okna.nakly[zap] > 0) h=-1;             
   if(okna.nakly[zap] < 0) h=1;              
   if(okna.nakly[zap]== 0) h=0;              
   if((h!=0)&&(v!=0))  
   {
    p=(okna.st_pp[zap+h]*labs(okna.nakly[zap]))+(okna.st_pp[zap+v]*labs(okna.naklx[zap]));
    p=okna.st_pp[zap]+(p/vson);
   }
   else if((h==0)&&(v!=0)) 
   {
    p=okna.st_pp[zap]+okna.st_pp[zap+v];
   }
   else if((h!=0)&&(v==0)) 
   {
    p=okna.st_pp[zap]+okna.st_pp[zap+h];
   }
   else if((h==0)&&(v==0)) 
   {
    p=0;
   }
   if(p>((par_ed.okno_vel)*3)/4)
   {
    ++hfeatures;
    if(okna.naklx[zap]==0)
    {
     strtx=okna.tockx[zap];
     strty=0;
     smerx=1;
     smery=1; 
     k=(float)(par_ed.okno_sir+1);   
    }
    else
    {
     x0=(float)okna.tockx[zap];
     y0=(float)okna.tocky[zap];
     k=-1*(float)okna.nakly[zap]/(float)okna.naklx[zap];
     
     i=0;
     if(k!=0) a=x0-(y0/k);  else a=-1;          
     if((a>=0)&&(a<=(float)(par_ed.okno_sir-1)))
     {
      strtx=(long)a;
      strty=0;
      if(k<0) {smerx=-1; k=k*-1;}  else smerx=1;
      smery=1;
      i=1;   
     }
     a=y0-(k*x0);            
     if(i==0) 
      if((a>=0)&&(a<=(float)(par_ed.okno_sir-1)))
      {
       strtx=0;
       strty=(long)a;
       smerx=1;
       if(k<=0) {smery=-1; k=k*-1;} else smery=1;
       i=1;  
      }
     a=(k*(float)(par_ed.okno_sir-1)) + y0 - (k*x0);    
     if(i==0) 
      if((a>=0)&&(a<=(float)(par_ed.okno_sir-1)))
      {
       strtx=par_ed.okno_sir-1;
       strty=(long)a;
       smerx=-1;
       if(k<=0) {smery=1; k=k*-1;} else smery=-1;
      }
    }
    i=1;   
    prvi='y';
    st=0;  
    if((k<1)&&(k>0)) {k=1/k; prvi='x';}           
    if(k==0) {k=(float)(par_ed.okno_sir+1);  prvi='x';}  
    par_ed.okno[(strty*par_ed.okno_sir)+strtx]=255;
    while((strtx>=0)&&(strtx<=par_ed.okno_sir-1)&&(strty>=0)&&(strty<=par_ed.okno_sir-1))
    {
     j=((long)k*i)-st; 
     if(prvi=='x')
     {
      for(n=0; n<j; n++)
      {
       strtx=strtx+smerx;
       if((strtx>=0)&&(strtx<=par_ed.okno_sir-1)) par_ed.okno[(strty*par_ed.okno_sir)+strtx]=255; else break;
      }
      strty=strty+smery;
      if((strty>=0)&&(strty<=par_ed.okno_sir-1)) par_ed.okno[(strty*par_ed.okno_sir)+strtx]=255; else break;
     }
     if(prvi=='y')
     {
      for(n=0; n<j; n++)
      {
       strty=strty+smery;
       if((strty>=0)&&(strty<=par_ed.okno_sir-1)) par_ed.okno[(strty*par_ed.okno_sir)+strtx]=255; else break;
      }
      strtx=strtx+smerx;
      if((strtx>=0)&&(strtx<=par_ed.okno_sir-1)) par_ed.okno[(strty*par_ed.okno_sir)+strtx]=255; else break;
     }
     st=st+j;   
     ++i;      
    }
   }
  }
 }
}
/**********************************************************************************************************************************/
void HandDetector::ED_(unsigned char* input_image, unsigned char* output_image)  
{
 long i,j,k,l,m,n,o,p;

 hfeatures=0;
 okna.st=0; 
 pix_tab.st=0;
 for(i=par_ed.start_ver; i<(par_ed.st_ver*par_ed.okno_pre)+par_ed.start_ver; i=i+par_ed.okno_pre)     
  for(j=par_ed.start_hor; j<(par_ed.st_hor*par_ed.okno_pre)+par_ed.start_hor; j=j+par_ed.okno_pre)    
  {
   k=(i*hor)+j;   
   n=0;
   for(l=0; l<par_ed.okno_sir; l++)
    for(m=0; m<par_ed.okno_sir; m++)
    {
     par_ed.okno[n]=input_image[k+(l*hor)+m];       
     ++n;
     }
   ED_PB();
   }
 o=0;
 for(i=par_ed.start_ver; i<(par_ed.st_ver*par_ed.okno_pre)+par_ed.start_ver; i=i+par_ed.okno_pre)     
  for(j=par_ed.start_hor; j<(par_ed.st_hor*par_ed.okno_pre)+par_ed.start_hor; j=j+par_ed.okno_pre)    
  {
   ED_DE(o);
   k=(i*hor)+j;   
   n=0;
   for(l=0; l<par_ed.okno_sir; l++)
    for(m=0; m<par_ed.okno_sir; m++)
    {
     p=k+(l*hor)+m;     
     if(par_ed.okno[n]==255) 
     {
      output_image[p]=255;            
      pix_tab.tab_v[pix_tab.st]=255;  
      pix_tab.tab_p[pix_tab.st]=p;    
      ++pix_tab.st;                   
     }
     else output_image[p]=0; 
     ++n;
     }
   ++o;
   }
}
/**********************************************************************************************************************************/
void HandDetector::DT_(unsigned char* output_image)  
{
 long i, st, p_zg, p_sp, p_le, p_de, p_zl, p_zd, p_sl;
 long p_sd, razlika, start,velikost,m,h_1;
 unsigned char a,b;

 h_1=hor-1;
 velikost=hor*ver;
 razlika=1;
 st=pix_tab.st;
 start=0;
 for(i=0; i<velikost; i++) output_image[i]=image_ed[i];  
 while(razlika!=0)
 {
  for(i=start; i<pix_tab.st; i++)
  {
   if(pix_tab.tab_v[i]>=7) 
   {
    p_zg=pix_tab.tab_p[i]+hor;
    p_sp=pix_tab.tab_p[i]-hor;
    p_de=pix_tab.tab_p[i]+1;
    p_le=pix_tab.tab_p[i]-1;
    p_zd=p_zg+1;
    p_zl=p_zg-1;
    p_sd=p_sp+1;
    p_sl=p_sp-1;
    m=pix_tab.tab_p[i]%hor;      
    a=pix_tab.tab_v[i]-5;
    b=a-2;
    if((p_zd<velikost)&&(output_image[p_zg]==0)) 
    {
     output_image[p_zg]=a;       
     pix_tab.tab_v[st]=a;        
     pix_tab.tab_p[st]=p_zg;
     ++st;
     if((output_image[p_zd]==0)&&(m!=h_1)) 
     { 
      output_image[p_zd]=b;       
      pix_tab.tab_v[st]=b;        
      pix_tab.tab_p[st]=p_zd;
      ++st;
     }
    }
    if((p_sl>=0)&&(output_image[p_sp]==0))
    {
     output_image[p_sp]=a;       
     pix_tab.tab_v[st]=a;        
     pix_tab.tab_p[st]=p_sp;
     ++st;
     if((output_image[p_sl]==0)&&(m!=0))
     {
      output_image[p_sl]=b;       
      pix_tab.tab_v[st]=b;        
      pix_tab.tab_p[st]=p_sl;
      ++st;
     } 
    }
    if((m!=0)&&(output_image[p_le]==0))
    {
     output_image[p_le]=a;       
     pix_tab.tab_v[st]=a;        
     pix_tab.tab_p[st]=p_le;
     ++st;
     if((output_image[p_zl]==0)&&(p_zl<velikost)) 
     {
      output_image[p_zl]=b;      
      pix_tab.tab_v[st]=b;       
      pix_tab.tab_p[st]=p_zl;
      ++st;
     }
    }
    if((m!=h_1)&&(output_image[p_de]==0))
    {
     output_image[p_de]=a;       
     pix_tab.tab_v[st]=a;       
     pix_tab.tab_p[st]=p_de;
     ++st;
     if((output_image[p_sd]==0)&&(p_sd>=0))
     {
      output_image[p_sd]=b;      
      pix_tab.tab_v[st]=b;       
      pix_tab.tab_p[st]=p_sd;
      ++st;
     }
    }
   }
  }
  razlika=st-pix_tab.st;
  start=pix_tab.st;
  pix_tab.st=st;
 }
}
/**********************************************************************************************************************************/
void HandDetector::MT_(unsigned char* polje)  
{
 long min_hor,max_hor, delta_hor;           
 long min_ver, max_ver, delta_ver;          
 long min_kot, max_kot, delta_kot;          
 long min_vel, max_vel, delta_vel;          
 long maxvelv,maxvel;       
 long h,i,j,k,l,m,n,o;
 long ver_m_2, hor_m_2;   
 long phor,pver;          
 long az,z;
 long p;                  
 long stzad;              
 long kk,sinus,cosinus;   
 unsigned char aa,bb;     
 unsigned char max_koef;  
 float aaf; 

 hor_m_2=hor/2;
 delta_hor=hor/(e_mod.razl_hor+1);
 min_hor=delta_hor;
 max_hor=delta_hor*e_mod.razl_hor;
 ver_m_2=ver/2;
 delta_ver=ver/(e_mod.razl_ver+1);
 min_ver=delta_ver;
 max_ver=delta_ver*e_mod.razl_ver;
 delta_kot=(e_mod.max_kot-e_mod.min_kot)/(e_mod.razl_kot-1);    
 min_kot=e_mod.min_kot;
 max_kot=e_mod.max_kot;
 min_vel=32512/(e_mod.min_vel/2);     
 max_vel=32512/(e_mod.max_vel/2);     
 z=8;  
 aaf=(((float)e_mod.razl_vel)*((float)max_vel))/((float)(min_vel-max_vel));
 delta_vel=(long)((float)min_vel*(float)z*aaf);   
 az=(long)((float)z*aaf);
 max_koef=0;
 res_mod.koef_u_mod[0]=0;
 res_mod.st_modelov=0;
 for(i=min_ver; i<=max_ver; i=i+delta_ver)    
 {
  maxvelv=abs(ver-i);        
  if(i<ver_m_2) maxvelv=i;   
  for(j=min_hor; j<=max_hor; j=j+delta_hor)   
  {
   maxvel=abs(hor-j);           
   if(j<hor_m_2) maxvel=j;      
   if(maxvelv<maxvel) maxvel=maxvelv;  
   maxvel=32512/maxvel;   
   if(maxvel<max_vel) maxvel=max_vel; 
   for(l=min_kot; l<=max_kot; l=l+delta_kot)   
   {
    kk=l;                                              
    if(kk<0) kk=kk+1000;         
    sinus=e_mod.sin0_PI_2[kk];  
    kk=kk+250;                  
    if(kk>=1000) kk=kk-1000;    
    cosinus=e_mod.sin0_PI_2[kk]; 
    m=z;
    for(k=min_vel; k>maxvel; m=m+z)  
    {
     aa=255;  
     stzad=0; 
     for(h=0; h<e_mod.st; h++)
     {
      phor=(cosinus * e_mod.p_hor[h]) - ( sinus   * e_mod.p_ver[h]);
      pver=(sinus   * e_mod.p_hor[h]) + ( cosinus * e_mod.p_ver[h]);
      phor=phor/k;
      pver=pver/k;
      phor=phor+j;
      pver=pver+i;
      p=(hor*pver)+phor;    
      bb=polje[p];
      if(bb<1) {aa=0; break;}   
      if(bb<aa) aa=bb;
      if(bb==255) ++stzad; 
     }
     if(aa>max_koef)
     {
      n=res_mod.st_modelov; 
      while((n>=0) && (aa>res_mod.koef_u_mod[n])) --n;
      ++n; 
      if(res_mod.st_modelov<(res_mod.max_modelov-1)) ++res_mod.st_modelov;
      for(o=res_mod.st_modelov-1; o>=n; o--) 
      {
       res_mod.polozaj_hor[o+1]=res_mod.polozaj_hor[o];
       res_mod.polozaj_ver[o+1]=res_mod.polozaj_ver[o];
       res_mod.kot[o+1]=res_mod.kot[o];
       res_mod.velikost[o+1]=res_mod.velikost[o];
       res_mod.koef_u_mod[o+1]=res_mod.koef_u_mod[o];
      }
      res_mod.polozaj_hor[n]=j;
      res_mod.polozaj_ver[n]=i;
      res_mod.kot[n]=l;
      res_mod.velikost[n]=k;
      res_mod.koef_u_mod[n]=aa;
      max_koef=res_mod.koef_u_mod[res_mod.st_modelov];
     }
     k=delta_vel/(az+m);
    }
   }
  }
 }
 if(res_mod.koef_u_mod[0]>0) ++res_mod.st_modelov;  
}
/**********************************************************************************************************************************/
void HandDetector::MA_(unsigned char* polje)  
{
 long h,i,j,k;
 long px1,py1,ko1,ve1;
 long px2,py2,ko2,ve2;                   
 long px, py, ko, ve;
 long pho,pve;
 long ve_, ve_z, ve_s;
 long kk,sinus, cosinus;
 long d1,d2,d3,d4;
 long p,vs;
 unsigned char koef,koef_min,aa;

 vs=hor*ver;
 for(i=0; i<res_mod.st_modelov; i++)  
 {
  koef_min=0;
  for(j=0; j<e_mod.st_iteracij; j++)  
  {
   ve_=32512/res_mod.velikost[i];
   if(ve_<15) break;   
   ve_=(long)((float)ve_*e_mod.povp);
   ve_s=ve_-1;
   ve_z=ve_+1;
   ve_=32512/ve_;
   ve_s=32512/ve_s;
   ve_s=ve_s-ve_;
   ve_z=32512/ve_z;
   ve_z=ve_z-ve_;
   for(px1=-1; px1<=1; px1++)
   {
    px=res_mod.polozaj_hor[i]+px1;
    for(py1=-1; py1<=1; py1++)
    {
     py=res_mod.polozaj_ver[i]+py1;
     for(ko1=-3; ko1<=3; ko1=ko1+3) 
     {
      ko=res_mod.kot[i]+ko1;
      kk=ko;                                             
      if(kk<0) kk=kk+1000;         
      sinus=e_mod.sin0_PI_2[kk];  
      kk=kk+250;                   
      if(kk>=1000) kk=kk-1000;     
      cosinus=e_mod.sin0_PI_2[kk]; 
      for(k=0; k<3; k++)
      {
       if(k==0) ve1=ve_z;
       if(k==1) ve1=0;
       if(k==2) ve1=ve_s;
       ve=res_mod.velikost[i]+ve1;
       koef=255;
       for(h=0; h<e_mod.st; h++)
       {
        pho=(cosinus * e_mod.p_hor[h]) - ( sinus   * e_mod.p_ver[h]);
        pve=(sinus   * e_mod.p_hor[h]) + ( cosinus * e_mod.p_ver[h]);
        pho=pho/ve;
        pve=pve/ve;
        pho=pho+px;
        pve=pve+py;
        if((pho<0)||(pho>=hor)){koef=0; break;} 
        if((pve<0)||(pve>=ver)){koef=0; break;} 
        p=(hor*pve)+pho;    
        aa=polje[p];
        if(aa<koef) koef=aa; 
        if(koef<koef_min) break; 
       }
       if(koef>koef_min)
       {
        px2=px1;
        py2=py1;
        ko2=ko1;
        ve2=ve1;
        koef_min=koef;
       }
      }
     }
    }
   }
   if((px2==0)&&(py2==0)&&(ko2==0)&&(ve2==0)) koef_min=0;
   while(koef_min>res_mod.koef_u_mod[i])  
   {
    res_mod.polozaj_hor[i]=res_mod.polozaj_hor[i]+px2;
    res_mod.polozaj_ver[i]=res_mod.polozaj_ver[i]+py2;
    res_mod.kot[i]=res_mod.kot[i]+ko2;
    if(res_mod.kot[i]>=500) res_mod.kot[i]=res_mod.kot[i]-1000;   
    if(res_mod.kot[i]<-500) res_mod.kot[i]=res_mod.kot[i]+1000;   
    res_mod.velikost[i]=res_mod.velikost[i]+ve2;
    res_mod.koef_u_mod[i]=koef_min;
    px=res_mod.polozaj_hor[i]+px2;
    py=res_mod.polozaj_ver[i]+py2;
    kk=res_mod.kot[i]+ko2;
    ve=res_mod.velikost[i]+ve2;
    if(kk<0) kk=kk+1000;            
    sinus=e_mod.sin0_PI_2[kk];     
    kk=kk+250;                      
    if(kk>=1000) kk=kk-1000;        
    cosinus=e_mod.sin0_PI_2[kk];   
    koef_min=255;
    for(h=0; h<e_mod.st; h++)
    {
     pho=(cosinus * e_mod.p_hor[h]) - ( sinus   * e_mod.p_ver[h]);
     pve=(sinus   * e_mod.p_hor[h]) + ( cosinus * e_mod.p_ver[h]);
     pho=pho/ve;
     pve=pve/ve;
     pho=pho+px;
     pve=pve+py;
     if((pho<0)||(pho>=hor)){koef_min=0; break;} 
     if((pve<0)||(pve>=ver)){koef_min=0; break;} 
     p=(hor*pve)+pho;    
     aa=polje[p];
     if(aa<koef_min) koef_min=aa; 
    }
   }
  }
 }
 if(res_mod.st_modelov>0)
 {
  for(i=0; i<res_mod.st_modelov-1; i++)
   for(j=i+1; j<res_mod.st_modelov; j++)
   {
    d1=abs(res_mod.polozaj_hor[i]-res_mod.polozaj_hor[j]);
    d2=abs(res_mod.polozaj_ver[i]-res_mod.polozaj_ver[j]);
    d3=abs(res_mod.kot[i]-res_mod.kot[j]);
    d4=abs(res_mod.velikost[i]-res_mod.velikost[j]);
    if((d1<6)&&(d2<6)&&(d3<20)&&(d4<150)) 
    {
     if(res_mod.koef_u_mod[i]<=res_mod.koef_u_mod[j]) res_mod.koef_u_mod[i]=0;
     else res_mod.koef_u_mod[j]=0;
    }
   }
  for(i=0; i<res_mod.st_modelov-1; i++)
   for(j=i+1; j<res_mod.st_modelov; j++)
    if(res_mod.koef_u_mod[i]<res_mod.koef_u_mod[j])
    {
     koef=res_mod.koef_u_mod[j];
     res_mod.koef_u_mod[j]=res_mod.koef_u_mod[i];
     res_mod.koef_u_mod[i]=koef;

     h=res_mod.polozaj_hor[j];
     res_mod.polozaj_hor[j]=res_mod.polozaj_hor[i];
     res_mod.polozaj_hor[i]=h;
     h=res_mod.polozaj_ver[j];
     res_mod.polozaj_ver[j]=res_mod.polozaj_ver[i];
     res_mod.polozaj_ver[i]=h;
     h=res_mod.velikost[j];
     res_mod.velikost[j]=res_mod.velikost[i];
     res_mod.velikost[i]=h;
     h=res_mod.kot[j];
     res_mod.kot[j]=res_mod.kot[i];
     res_mod.kot[i]=h;
    }
  for(i=0; i<res_mod.st_modelov; i++)
   if(res_mod.koef_u_mod[i]==0) {res_mod.st_modelov=i; break;} 
 }
}
/**********************************************************************************************************************************/
void HandDetector::SR_(unsigned char *input_image, unsigned char* output_image) 
{
 long i,p,kk;
 long j,x,y,v;
 long phor, pver;
 long sinus, cosinus;

 for(i=0; i<3*hor*ver; i++) output_image[i]=input_image[i];
 if(res_mod.st_modelov>0) res_mod.st_modelov=1;  
 for(j=0; j<res_mod.st_modelov; j++)
 {
  x=res_mod.polozaj_hor[j];
  y=res_mod.polozaj_ver[j];
  kk=res_mod.kot[j];
  v=res_mod.velikost[j];
  if(kk<0) kk=kk+1000;           
  sinus=e_mod.sin0_PI_2[kk];     
  kk=kk+250;                     
  if(kk>=1000) kk=kk-1000;       
  cosinus=e_mod.sin0_PI_2[kk];   
  for(i=0; i<e_mod.st; i++)
  {
   phor=(cosinus * e_mod.p_hor[i]) - ( sinus   * e_mod.p_ver[i]);
   pver=(sinus   * e_mod.p_hor[i]) + ( cosinus * e_mod.p_ver[i]);
   phor=phor/v;
   pver=pver/v;
   phor=phor+x;
   pver=pver+y;
   p=(3*hor*pver)+(3*phor); 
   output_image[p]=0;                            
   output_image[p+1]=255;
   output_image[p+2]=255;
  }
 }
}
/**********************************************************************************************************************************/
HandDetector::HandDetector(long hres, long vres, unsigned char *iimage, char *color_file_name)
{
 long i;

 hor=hres;
 ver=vres;
 LC_(color_file_name);
 par_ed.okno_sir=12;  
 par_ed.okno_pre=par_ed.okno_sir/2;
 par_ed.okno_vel=par_ed.okno_sir*par_ed.okno_sir;
 par_ed.start_hor=(hor%(hor/par_ed.okno_pre))/2; 
 par_ed.start_ver=(ver%(ver/par_ed.okno_pre))/2; 
 par_ed.st_hor=(hor/par_ed.okno_pre)-1;          
 par_ed.st_ver=(ver/par_ed.okno_pre)-1;          
 image_in=iimage;
 image_se=new unsigned char[hor*ver];   
 image_ed=new unsigned char[hor*ver];   
 image_ds=new unsigned char[hor*ver];  
 image_re=new unsigned char[3*hor*ver]; 
 okna.naklx=new long [par_ed.st_hor*par_ed.st_ver];
 okna.nakly=new long [par_ed.st_hor*par_ed.st_ver];
 okna.tockx=new long [par_ed.st_hor*par_ed.st_ver];
 okna.tocky=new long [par_ed.st_hor*par_ed.st_ver];
 okna.st_pp=new long [par_ed.st_hor*par_ed.st_ver];
 par_ed.okno=new unsigned char [par_ed.okno_vel];
 par_ed.index_okno=new unsigned char [par_ed.okno_vel]; 
 for(i=0; i<par_ed.okno_vel; i++) par_ed.index_okno[i]=0;
 for(i=1; i<par_ed.okno_sir-1; i++) par_ed.index_okno[i]=1;
 for(i=(par_ed.okno_sir*(par_ed.okno_sir-1))+1; i<(par_ed.okno_vel)-1; i++) par_ed.index_okno[i]=2;
 for(i=par_ed.okno_sir; i<par_ed.okno_sir*(par_ed.okno_sir-1); i=i+par_ed.okno_sir) par_ed.index_okno[i]=3;
 for(i=(2*par_ed.okno_sir)-1; i<(par_ed.okno_vel)-1; i=i+par_ed.okno_sir) par_ed.index_okno[i]=4;
 par_ed.index_okno[0]=5;
 par_ed.index_okno[par_ed.okno_sir-1]=6;
 par_ed.index_okno[par_ed.okno_sir*(par_ed.okno_sir-1)]=7;
 par_ed.index_okno[(par_ed.okno_vel)-1]=8;
 pix_tab.tab_v=new unsigned char [hor*ver];
 pix_tab.tab_p=new long [hor*ver];
 LM_();
 e_mod.razl_hor=hor/15;  
 e_mod.razl_ver=ver/15;  
 e_mod.min_vel=110;       
 e_mod.max_vel=150;       
 e_mod.razl_vel=3;       
 e_mod.min_kot=-499;     
 e_mod.max_kot=499;    
 e_mod.razl_kot=20;     
 e_mod.st_iteracij=4;
 res_mod.max_modelov=20;      
 res_mod.polozaj_hor=new long [res_mod.max_modelov];
 res_mod.polozaj_ver=new long [res_mod.max_modelov];
 res_mod.kot=new long [res_mod.max_modelov];
 res_mod.velikost=new long [res_mod.max_modelov];
 res_mod.koef_u_mod=new unsigned char [res_mod.max_modelov];
}
/**********************************************************************************************************************************/
HandDetector::~HandDetector(void)
{
 delete [] image_se;
 delete [] image_ed;
 delete [] image_ds;
 delete [] image_re;
 delete [] okna.naklx;
 delete [] okna.nakly;
 delete [] okna.tockx;
 delete [] okna.tocky;
 delete [] okna.st_pp;
 delete [] par_ed.okno;
 delete [] par_ed.index_okno;
 delete [] pix_tab.tab_v;
 delete [] pix_tab.tab_p;
 delete [] e_mod.pix;
 delete [] e_mod.p_hor;
 delete [] e_mod.p_ver;
 delete [] e_mod.sin0_PI_2;
 delete [] res_mod.polozaj_hor;
 delete [] res_mod.polozaj_ver;
 delete [] res_mod.kot;
 delete [] res_mod.velikost;
 delete [] res_mod.koef_u_mod;
}
/**********************************************************************************************************************************/
void HandDetector::Run(void)
{
 CS_(image_in,image_se);
 ED_(image_se,image_ed);
 DT_(image_ds);
 MT_(image_ds);
 MA_(image_ds);
 SR_(image_in,image_re);
}
/**********************************************************************************************************************************/
