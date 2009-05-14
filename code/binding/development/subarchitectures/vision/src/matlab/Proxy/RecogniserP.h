#include <vision/idl/Vision.hh>

extern void R_LoadAvModels(const char* filename);
extern void R_RunComponent(int fl);
extern Vision::RecognisedAttributes* R_Recognise(const Vision::ROI &Roi);
extern void R_Update(const Vision::LearnInstruction &learnInstruction, const Vision::ROI &Roi);
