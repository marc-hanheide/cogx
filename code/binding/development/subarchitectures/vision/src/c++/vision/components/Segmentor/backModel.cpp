//---------------------------------------------------------------------------
#include "backModel.h"
#include "string.h"
#include "math.h"


#ifdef __APPLE__
#include <float.h>
#define MINFLOAT FLT_MIN
#else
#include "values.h"
#endif

//#define SELECTIVE_ADAPT
//
#define ORTHRESHOLD
//
#define CONSTANT_INCREMENT
//---------------------------------------------------------------------------

_back_model:: _back_model()
{
 phis = NULL ;
 current_mean = NULL ;
 current_var = NULL ;
 current_prc = NULL ;
 A_division = NULL ;

 W = 0 ;
 H = 0 ;
 lenWH = 0 ;
 isEmpty = 1 ;
 min_var_allowed = 3.0*3.0 ;
 N_channels = 3 ;
 aux_mu = new float [N_channels] ;
 aux_var = new float [N_channels] ;
 compScale = 2.5*2.5 ;
 N_norm = 0 ;
 min_val_accepted=0.0 ;
 phi_CD = 0.0 ;
 phi_CD_px = 1.0 ;

 // default values on bounds
 global_delta_alpha_default = 0.7 ; 0.5 ;
 global_ulim_cromacy_default = 15.0 ;

 // since ve compare squared for efficiency
 global_ulim_cromacy_default *= global_ulim_cromacy_default ;

 global_delta_alpha = global_delta_alpha_default ;
 global_ulim_cromacy = global_ulim_cromacy_default ;
}

_back_model:: ~_back_model()
{
 deleteImages() ;
 if ( aux_mu != NULL ) delete [] aux_mu ;
 if ( aux_var != NULL ) delete [] aux_var ;
 aux_mu = NULL ;
 aux_var = NULL ;
}

void _back_model:: deleteImages()
{
 if ( phis != NULL ) delete [] phis ;
 phis = NULL ;
 if ( current_mean != NULL ) delete [] current_mean ;
 current_mean = NULL ;
 if ( current_var != NULL ) delete [] current_var ;
 current_var = NULL ;
 if ( current_prc != NULL ) delete [] current_prc ;
 current_prc = NULL ;
 if ( A_division != NULL ) delete [] A_division ;
 A_division = NULL ;
 phi_CD = 0.0 ;
 isEmpty = 1 ;
 N_norm = 0 ;
 phi_CD_px = 1.0 ;
}


void _back_model:: createImages( int wW, int hH )
{
 deleteImages() ;
 W = wW ;
 H = hH ;
 lenWH = W*H ;
 phis = new float [wW*hH] ;
 current_mean = new float [wW*hH*N_channels] ;
 current_var = new float [wW*hH*N_channels] ;
 current_prc = new float [wW*hH*N_channels] ;
 A_division = new float [wW*hH] ;

 memset( phis, 0, lenWH*sizeof(float) ) ;
 memset( current_mean, 0, lenWH*N_channels*sizeof(float) ) ;
 memset( current_var, 0, lenWH*N_channels*sizeof(float) ) ;
 memset( current_prc, 0, lenWH*N_channels*sizeof(float) ) ;
 memset( A_division, 0, lenWH*sizeof(float) ) ;
}

void _back_model:: resetData()
{
 deleteImages() ;
}

void _back_model:: insertImage( unsigned char *image, int W, int H, int type_of_update )
{
 unsigned char *ptr_input ;
 float *ptr_phis ;
 float *ptr_mean ;
 float *ptr_var, *ptr_prc ;
 int i, j, k ;
 float x, s, s2, phi_N, phi_x ;
 float m_c, v_c ;
 int skip ;
 float new_impact, old_impact ;

 #ifdef SELECTIVE_ADAPT
    type_of_update = 1 ;
 #endif

 new_impact = 0.00 ;
 old_impact = 1.0 - new_impact ;

 if ( isEmpty == 1 )  {
   createImages( W, H ) ;
 }

 ptr_input = image ;
 ptr_phis = phis ;
 ptr_mean = current_mean ;
 ptr_var = current_var ;

 // if this is first insertion, no prob calculation is required
 if ( isEmpty == 1 ) {
    N_norm = 0 ;
    for ( i = 0 ; i < lenWH*N_channels ; ++i ) {
        *ptr_mean = (float)(*ptr_input) ;
        ptr_input++ ;
        ptr_mean++ ;
    }
    float init_var ;
    init_var = min_var_allowed ;
    for ( i = 0 ; i < lenWH ; ++i ) {
        *ptr_phis = MINFLOAT ; ptr_phis++ ;
        *ptr_var = init_var ; ptr_var++ ;
        *ptr_var = init_var ; ptr_var++ ;
        *ptr_var = init_var ; ptr_var++ ;
    }
    isEmpty = 0 ;
    phi_CD = 0.0 ;
    phi_CD_px = 1.0 ;
    ulim_cromacy = global_ulim_cromacy_default/9.0 ;
    return ;
 }

 // if this is not first insertion
 for ( i = 0 ; i < lenWH ; ++i ) {
     skip = 0 ;
     for ( j = 0 ; j < N_channels ; ++j ) {
        x = (float)*(ptr_input) ;
        m_c = *(ptr_mean+j) ;
        v_c = *(ptr_var+j) ;

        s = x - m_c ;
        s2 = s*s ;


        if ( v_c < min_var_allowed ) {
                v_c = min_var_allowed ;
        }

        #ifdef SELECTIVE_ADAPT
        if ( type_of_update == 1 )
           if ( s2 > 9.0*compScale*v_c ) {
              skip = 1 ;
              k = N_channels - j ;
              ptr_input += k ;
              break ;
           }
        }
        #endif

        #ifdef CONSTANT_INCREMENT
           phi_x = 1.0 ;
        #else
           phi_x = exp(-0.5*s2/v_c)/v_c ;
        #endif
        phi_N = *ptr_phis + phi_x ;
        aux_mu[j] = old_impact*((x*phi_x + m_c*(*ptr_phis))/phi_N)  + new_impact * x ;
        aux_var[j] = old_impact*((s2*phi_x + v_c*(*ptr_phis))/phi_N) + new_impact * s2 ;
        ptr_input++ ;
     }
     // if decided to adapt the model
     if ( skip == 0 ) {
        for ( j = 0 ; j < N_channels ; ++j ) {
            *ptr_mean = aux_mu[j] ;
            *ptr_var =  aux_var[j] ;
            ptr_mean++ ;
            ptr_var++ ;
        }
        *ptr_phis = phi_N ;
        ptr_phis++ ;
     } else {
        ptr_mean += N_channels ;
        ptr_var += N_channels ;
        ptr_phis++ ;
     }
 }

}


void _back_model:: addImageToNorms( unsigned char *input )
{
 float *ptr_mean, *ptr_var ;
 unsigned char *ptr_input ;
 float An, alpha_i, al_val, CD ;
 int i,j ;
 float x ;

 float var_null = 0.0 ;
 ptr_var = current_var ;
 for ( i = 0 ; i < lenWH*N_channels ; ++i ) {
     var_null += *ptr_var ;
     ptr_var++ ;
 } var_null /= lenWH ;
 min_val_accepted = var_null*3.0*3.0 ;

 ptr_input = input ;
 ptr_mean = current_mean ;
 for ( i = 0 ; i < lenWH ; ++i ) {
     An = 0.0 ;
     alpha_i = 0 ;
     for ( j = 0 ; j < N_channels ; ++j ) {
         alpha_i += (*(ptr_mean+j))*((float)(*(ptr_input+j))) ;
         An += (*(ptr_mean+j))*(*(ptr_mean+j)) ;
     }
     if ( An < min_val_accepted )  {
        alpha_i = 1.0 ;
        ptr_mean += 3 ;
        ptr_input += 3 ;
        continue ;
     }
     alpha_i /= An ;

     al_val = fabs(1.0 - alpha_i) ;
     if ( al_val < global_delta_alpha_default ) {
        CD = 0.0 ;
        for ( j = 0 ; j < N_channels ; ++j ) {
         x = ((float)*(ptr_input+j)) - alpha_i*(*(ptr_mean+j)) ;
         CD += x*x ;
        }

        ulim_cromacy = (CD*phi_CD_px + ulim_cromacy* phi_CD)/(phi_CD+phi_CD_px) ;
        phi_CD += phi_CD_px ;
      //  N_norm++ ;
     }

     ptr_mean += 3 ;
     ptr_input += 3 ;
 }

  if ( phi_CD == 0 ) phi_CD = 0.0001 ;

  phi_CD_px /= phi_CD ;
  phi_CD /= phi_CD ;

 global_ulim_cromacy = ulim_cromacy *9.0*10.0 ;
}

void _back_model:: generateNorms()
{
 if ( N_norm == 0 ) {
     global_delta_alpha = global_delta_alpha_default ;
     global_ulim_cromacy = global_ulim_cromacy_default ;
     non_modified_alpha = global_delta_alpha ;
     return ;
 }
// global_ulim_cromacy /= N_norm ;
 global_delta_alpha = global_delta_alpha_default ;

// global_ulim_cromacy = sqrt(global_ulim_cromacy)*3.0 ;
// global_ulim_cromacy *= global_ulim_cromacy ;

// global_ulim_cromacy = min_val_accepted ;
 non_modified_alpha = global_delta_alpha ;
// N_norm = 0 ;
// isEmpty = 1 ;
}

void _back_model:: shadowIntensitySensitivity( float shd )
{
 global_delta_alpha = shd ;
}

void _back_model:: generateConstants( int calculate_thresholds )
{
 float *ptr_var, *ptr_prc, *ptr_mean, *ptr_A_division ;
 int i, j ;

 // generate constants for background subtraction
 min_val_accepted = 0.0 ;
 ptr_var = current_var ;
 ptr_prc = current_prc ;
 for ( i = 0 ; i < lenWH*N_channels ; ++i ) {
     #ifdef ORTHRESHOLD
        *ptr_prc = (*ptr_var)*compScale*2.0 ;
     #else
        *ptr_prc = 1.0 / *ptr_var ;
     #endif
     min_val_accepted += *ptr_var ;
     ptr_var++ ;
     ptr_prc++ ;
 }
  min_val_accepted = (min_val_accepted/(float)lenWH)*3.0*3.0 ;

 // if generate thresholds or use the defaults
 if ( calculate_thresholds == 0 ) { N_norm = 0; } else { N_norm = 1.0 ; } 
 generateNorms() ;

 // generate division factors for normalizing alphas
 ptr_A_division = A_division ;
 ptr_mean = current_mean ;
 for ( i = 0 ; i < lenWH ; ++i ) {
     *ptr_A_division = 0.0 ;
     for ( j = 0 ; j < N_channels ; ++j ) {
         *ptr_A_division += (*ptr_mean)*(*ptr_mean) ;
         ptr_mean++ ;
    }
    if ( *ptr_A_division < min_var_allowed ) {
        *ptr_A_division = min_var_allowed ;
    }
    *ptr_A_division = 1.0 / *ptr_A_division ;
    ptr_A_division++ ;
 }

}

void _back_model:: getBackground( unsigned char *image, int W, int H )
{
 unsigned char *ptr_input ;
 float *ptr_mean ;

 if ( W*H != lenWH ) return ;

 ptr_mean = current_mean ;
 ptr_input = image ;
 for ( int i = 0 ; i < lenWH*N_channels ; ++i ) {
     *ptr_input = (unsigned char)*ptr_mean ;
      ptr_mean++ ;
      ptr_input++ ;
 }
}

void _back_model:: getMaskFunction(unsigned char *input, unsigned char *output  )
{
 unsigned char mask_val ;
 unsigned char *ptr_output ;
 unsigned char *ptr_input ;
 float *ptr_var ;
 float *ptr_mean ;
 float *ptr_prc, *ptr_A_division ;
 int i, j ;
 float  s, s2 ;
 float distance ;
 float alpha, alpha2 ;
 float p_dist ;


 float length_cur ;

 ptr_var = current_var ;
 ptr_output = output ;
 ptr_input = input ;
 ptr_mean = current_mean ;
 ptr_prc = current_prc ;
 ptr_A_division = A_division ;
 for ( i = 0 ; i < lenWH ; ++i ) {
     mask_val = 0 ;
     distance = 0.0 ;
     p_dist = 0.0 ;
     alpha = 0.0 ;
     for ( j = 0 ; j < N_channels ; ++j ) {
        s = (float)(*(ptr_input+j)) - *(ptr_mean+j);
        s2 = s*s ;
        #ifdef ORTHRESHOLD
        if ( s2 > *(ptr_prc+j) ) { mask_val = 255 ; }
        #else
          distance += s2*(*(ptr_prc+j)) ;
        #endif
        alpha += (*(ptr_mean+j))*((float)(*(ptr_input+j))) ;
     }
     #ifndef ORTHRESHOLD
        if ( distance > 18.75 ) mask_val = 255 ;
     #endif

     // if the pixel is assigned to foreground
     if ( mask_val == 255 ) {
        alpha *= *ptr_A_division ;
        p_dist = 0.0 ;
        for ( j = 0 ; j < N_channels ; ++j ) {
                s = (float)(*ptr_input) - alpha*(*ptr_mean) ;
                p_dist += s*s ;
                ptr_input++ ;
                ptr_mean++ ;
                ptr_prc++ ;
        }


        alpha2 = fabs( 1.0 - alpha ) ;
    /**/    if ( alpha2 < global_delta_alpha &&
                p_dist < global_ulim_cromacy ) mask_val = 0 ;
      /**/
    } else {
       ptr_input += N_channels ;
       ptr_mean += N_channels ;
       ptr_prc += N_channels ;
    }
    *ptr_output = mask_val ;
    
    ptr_output++ ;
    ptr_A_division++ ;
    ptr_var += 3 ;
 }
}

void _back_model:: getMaskFunctionWithAdapt(unsigned char *input, unsigned char *output, int should_adapt_model0 )
{
 unsigned char mask_val ;
 unsigned char *ptr_output ;
 unsigned char *ptr_input ;
 float *ptr_var ;
 float *ptr_mean ;
 float *ptr_prc, *ptr_A_division ;
 float  *ptr_phis ;
 int i, j ;
 float  s, s2 ;
 float distance ;
 float alpha, alpha2 ;
 float p_dist ;
 int should_adapt_model ;
 float phi_x, phi_N ;

 float length_cur ;
 float new_impact, old_impact ;
 new_impact = 0.0 ;
 old_impact = 1.0 - new_impact ;

 ptr_phis = phis ;
 ptr_var = current_var ;
 ptr_output = output ;
 ptr_input = input ;
 ptr_mean = current_mean ;
 ptr_prc = current_prc ;
 ptr_A_division = A_division ;
 for ( i = 0 ; i < lenWH ; ++i ) {
     mask_val = 0 ;
     distance = 0.0 ;
     p_dist = 0.0 ;
     alpha = 0.0 ;
     for ( j = 0 ; j < N_channels ; ++j ) {
        s = (float)(*(ptr_input+j)) - *(ptr_mean+j);
        s2 = s*s ;
        #ifdef ORTHRESHOLD
        if ( s2 > *(ptr_prc+j) ) { mask_val = 255 ; }
        #else
          distance += s2*(*(ptr_prc+j)) ;
        #endif
        alpha += (*(ptr_mean+j))*((float)(*(ptr_input+j))) ;
     }
     #ifndef ORTHRESHOLD
        if ( distance > 18.75 ) mask_val = 255 ;
     #endif
     should_adapt_model = 0 ;

        alpha *= *ptr_A_division ;
        p_dist = 0.0 ;
        for ( j = 0 ; j < N_channels ; ++j ) {
                s = (float)(*(ptr_input+j)) - alpha*(*(ptr_mean+j)) ;
                p_dist += s*s ;
             /*   ptr_input++ ;
                ptr_mean++ ;
                ptr_prc++ ;*/
        }

       alpha2 = fabs( 1.0 - alpha ) ;
       if ( alpha2 < global_delta_alpha &&  p_dist < global_ulim_cromacy ) {
           mask_val = 0 ;
           should_adapt_model = should_adapt_model0 ;
       }

    if ( should_adapt_model == 1 ) {
       ulim_cromacy = (p_dist*phi_CD_px + ulim_cromacy* phi_CD)/(phi_CD+phi_CD_px)*old_impact + p_dist*new_impact ;
       phi_CD += phi_CD_px ;

       for ( j = 0 ; j < N_channels ; ++j ) {
            s2 = *ptr_mean - (float)*ptr_input ;
            s2 *= s2 ;
 
            float phi_x = 1.0 ;
            float phi_N = *ptr_phis + phi_x ;
            *ptr_mean  = old_impact*((*ptr_input*phi_x + (*ptr_mean) *(*ptr_phis))/phi_N) + new_impact * (*ptr_input) ;
            *ptr_var = old_impact*((s2*phi_x + (*ptr_var) *(*ptr_phis))/phi_N) + new_impact * s2 ;
            if( *ptr_var > 0 ) {
                *ptr_prc = (*ptr_var)*compScale*2.0 ;
            } else {
                *ptr_prc = 1.0 ;
            }
 
            ptr_mean ++ ;
            ptr_input ++ ;
            ptr_var ++ ;
            ptr_prc ++ ;

       }
    } else {
       ptr_prc += N_channels ;

       ptr_mean += N_channels ;
       ptr_input += N_channels ;
       ptr_var += N_channels ;

    }


    *ptr_output = mask_val ;

    ptr_output++ ;
    ptr_A_division++ ;

 }
}

void _back_model:: getImageSizes( int &Width, int &Height)
{
 Width = W ;
 Height = H ; 
}

void _back_model:: getLengthOfSaveArrayFloat( int &lengthSA ) 
{
 // the input is as all doubles	
 calculateLengthOfArrayFloat( lengthSA, W, H, N_channels ) ;	
}

void _back_model:: calculateLengthOfArrayFloat( int &lengthSA, int w, int h, int n ) 
{
 // the input is as all doubles	
 lengthSA = (1 + 1 + 1 + 1 + 1 + w*h*n*3 + w*h)  ;
 // w, h, Nchannel, g_alph, g_chrom, cur_var, cur_prc, cur_mean, A_div
	
}

void _back_model:: getFullArrayToSave( float *array, int &lengthSA )  
{
 float *ptr_array ;	
 float *ptr_current_mean, *ptr_current_var, *ptr_current_prc, *ptr_A_division ;	
 int counter ;
	
 ptr_current_mean = current_mean ; 
 ptr_current_var = current_var ;
 ptr_current_prc = current_prc ;
 ptr_A_division = A_division ;		
 ptr_array = array ;
 
 *ptr_array = (float)W ; ptr_array++ ; 
 *ptr_array = (float)H ; ptr_array++ ;      
 *ptr_array = (float)N_channels ; ptr_array++ ; 
 *ptr_array = (float)global_delta_alpha ; ptr_array++ ; 
 *ptr_array = (float)global_ulim_cromacy ; ptr_array++ ; 
 counter = 5 ;
 for ( int i = 0 ; i < lenWH*N_channels ; ++i ) {
 	*ptr_array = *ptr_current_mean ;
 	ptr_array++ ;
 	ptr_current_mean++ ;
 	counter++ ;
 }
 
 for ( int i = 0 ; i < lenWH*N_channels ; ++i ) {
 	*ptr_array = *ptr_current_var ;
 	ptr_array++ ;
 	ptr_current_var++ ;
 	counter++ ;
 } 
 
 for ( int i = 0 ; i < lenWH*N_channels ; ++i ) {
 	*ptr_array = *current_prc ;
 	ptr_array++ ;
 	current_prc++ ;
 	counter++ ;
 }  
 
 for ( int i = 0 ; i < lenWH ; ++i ) {
 	*ptr_array = *ptr_A_division ;
 	ptr_array++ ;
 	ptr_A_division++ ;
 	counter++ ;
 }   
 
 if ( counter != lengthSA ) lengthSA = -counter ; 
 	
}

void _back_model:: insertFullArray( float *array )  
{
 float *ptr_array ;	
 float *ptr_current_mean, *ptr_current_var, *ptr_current_prc, *ptr_A_division ;	
 
 ptr_array = array ;
 
 W = *ptr_array ; ptr_array++ ; 
 H = *ptr_array ; ptr_array++ ;      
 N_channels = *ptr_array ; ptr_array++ ; 

 createImages( W, H ) ;	
 global_delta_alpha = *ptr_array ; ptr_array++ ; 
 global_ulim_cromacy = *ptr_array ; ptr_array++ ; 	
	
 lenWH = W*H ; 
 ptr_current_mean = current_mean ; 
 ptr_current_var = current_var ;
 ptr_current_prc = current_prc ;
 ptr_A_division = A_division ;		

 for ( int i = 0 ; i < lenWH*N_channels ; ++i ) {
 	*ptr_current_mean = *ptr_array ;
 	ptr_array++ ;
 	ptr_current_mean++ ;
 }
 
 for ( int i = 0 ; i < lenWH*N_channels ; ++i ) {
 	*ptr_current_var = *ptr_array ;
 	ptr_array++ ;
 	ptr_current_var++ ;
 } 
 
 for ( int i = 0 ; i < lenWH*N_channels ; ++i ) {
 	*ptr_current_prc = *ptr_array ;
 	ptr_array++ ;
 	ptr_current_prc++ ;
 }  
 
 for ( int i = 0 ; i < lenWH ; ++i ) {
 	*ptr_A_division = *ptr_array ;
 	ptr_array++ ;
 	ptr_A_division++ ;
 }   
 	
}
//------------------------------------------------------------------------------
#undef SELECTIVE_ADAPT
#undef CONSTANT_INCREMENT
