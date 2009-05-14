//---------------------------------------------------------------------------
/*
  backModel.h
  Matej Kristan 2007

  Background modeling class. Used to generate a background model of the scene
  and automatically produce a mask function, to separate background from
  non-background objects.

  Learning:
        stage 1:
                 First use "insertImage" to insert a number of images in sequential
                 manner. Fore example, insert 30 consecutive images.
                 At this stage, a Gaussian is learnt for each pixel separately,
                assuming color channel independance for computational efficiency.
        stage 2 (optional):
                 Add new images of background using "addImageToNorms", so that
                 color variation (in terms of shadows) can be estimated. If one
                 chooses not to estimate these values, "addImageToNorms" may not
                 be run.
        stage 3:
                 Run "generateConstants" to generate constants and lookup tables.

  Processing:
        background image:
                To obtain the backgroun image, use "getBackground"
        mask function:
                To obtain the current mask function, use "getMaskFunction"
        to change shadow sensitivity:
                Use "shadowIntensitySensitivity" to scale the shadow sensitivity
*/


#ifndef backModelH
#define backModelH
//---------------------------------------------------------------------------

class _back_model {
   private:
        float *phis ;
        float *current_mean ;// mean values of all pixels
        float *current_var ; // varinces of all pixels
        float *current_prc ; // if ORTHRESHOLD is defined then scales are stored here.
                             // otherwise, precisions are stored here.
        float *A_division ;  // lookup table of scaling factors for delta alpha

        int W, H, lenWH, N_channels;
        float N_norm ;
        int isEmpty ;
        float min_var_allowed ;
        float *aux_mu ;
        float *aux_var ;
        float compScale ;                   // scale threshold on variance
        float min_val_accepted ;            // minimum intensity value considered
                                            // for processing

        float global_delta_alpha ;          // current global delta alpha
        float global_ulim_cromacy ;         // current limit on cromacy
        float global_delta_alpha_default ;  // default value on delta alpha
        float global_ulim_cromacy_default ; // default value on cromacy limit
        float ulim_cromacy ;
        float non_modified_alpha ;          // nonmoified delta alpha ascalculated
                                            // in the beginning     
        float phi_CD ;                      // global constant corresponding to 
        				    // number of points used for background
                                            // CD estimation
        float phi_CD_px ;

        void deleteImages() ;
        void createImages( int W, int H ) ;
        void generateNorms() ;
   public:
        _back_model() ;
        ~_back_model() ;

        void resetData() ;
        void insertImage( unsigned char *image, int W, int H, int type_of_update = 0 ) ;
        void addImageToNorms( unsigned char *image ) ;
        void shadowIntensitySensitivity( float shd ) ;
        void generateConstants( int calculate_thresholds ) ;
        void getBackground( unsigned char *image, int W, int H ) ;
        void getMaskFunction(unsigned char *input, unsigned char *output ) ;
        void getMaskFunctionWithAdapt(unsigned char *input, unsigned char *output, int should_adapt_model = 0 ) ;
        void getImageSizes( int &Width, int &Height) ;
        void getLengthOfSaveArrayFloat( int &lengthSA ) ;
        void calculateLengthOfArrayFloat( int &lengthSA, int w, int h, int n ) ;
        void getFullArrayToSave( float *array, int &lengthSA ) ;
        void insertFullArray( float *array ) ;
} ;

#endif
