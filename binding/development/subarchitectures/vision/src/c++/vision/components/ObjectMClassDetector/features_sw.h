#ifndef FEATURES_SW_H_
#define FEATURES_SW_H_

/*
 * 
 * Visitor to produce features from an image
 * 
 * on_new_Image is called at the beginning of every single scale image
 * on_get_FeatureVector is called on every window and is supposed to fill features*
 * while returning with the length of the feature vector
 * 
 */

class FeatureVisitor {
public:
	virtual ~FeatureVisitor() {};
	virtual void on_new_Scale(const unsigned char* img, int width, int height, int rowStep, float scale, float originAtFullScaleX, float originAtFullScaleY) = 0;
	virtual float* on_get_FeatureVector(int posX, int posY, int width, int height, float* features) = 0;
	virtual int on_get_featureDim() const = 0;
	virtual void on_new_Image(const unsigned char* img, int width, int height, int rowStep) = 0;
};


		
#endif /*FEATURES_H_*/
