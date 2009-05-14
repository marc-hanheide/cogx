/*
HandDetector.h
(2008) Peter Rulic

Hand detection class used to detetect pointing with right hand
*/
#ifndef HandDetectorH
#define HandDetectorH
class HandDetector
{
 private:
 long hor;  
 long ver;  
 void LC_(char *fname);   
 void LM_(void);   
 void CS_(unsigned char* input_image, unsigned char* output_image); 
 void ED_(unsigned char* input_image, unsigned char* output_image);     
 void ED_PB(void);   
 void ED_DE(long zap);   
 void DT_(unsigned char* output_image);  
 void MT_(unsigned char* polje);  
 void MA_(unsigned char* polje);  
 void SR_(unsigned char *input_image, unsigned char* output_image); 
 struct TARGET_COLOR  
 {
  float t_min, t_max;           
  float k_r, n_r;               
  float k_g, n_g;               
  float k_b, n_b;               
  float smv_r,smv_g,smv_b;      
  float elipsa_a, elipsa_b;     
  float k_elipsa_a, k_elipsa_b;  
 } color;
 struct OKNA  
 {
  long* naklx;   
  long* nakly;   
  long* tockx;   
  long* tocky;   
  long* st_pp;   
  long st;       
 } okna;
 struct PAR_ED  
 {
  long okno_sir;              
  long okno_pre;              
  long okno_vel;              
  long start_hor;             
  long st_hor;                 
  long start_ver;             
  long st_ver;                  
  unsigned char *okno;        
  unsigned char *index_okno;  
 } par_ed;
 struct PIX_TAB 
 {
  unsigned char* tab_v;    
  long* tab_p;           
  long st;               
 } pix_tab;
 struct EDGE_MODEL     
 {
  unsigned char* pix; 
  char* p_hor;        
  char* p_ver;        
  long* sin0_PI_2;    
  float povp;         
  long st;             
  long razl_hor;                     
  long razl_ver;                     
  long min_vel, max_vel, razl_vel;   
  long min_kot, max_kot, razl_kot;   
  long st_iteracij;    
 } e_mod;

 public:

 long hfeatures;
 struct RES_MODEL     
 {
  long* polozaj_hor;         
  long* polozaj_ver;         
  long* kot;                 
  long* velikost;            
  unsigned char* koef_u_mod; 
  long st_modelov;           
  long max_modelov;          
 } res_mod;
 unsigned char* image_in;  
 unsigned char* image_se;  
 unsigned char* image_ed;  
 unsigned char* image_ds;  
 unsigned char* image_re;  
 HandDetector(long hres, long vres, unsigned char *iimage, char *color_file_name);
 ~HandDetector(void);
 void Run(void); 
};

#endif
