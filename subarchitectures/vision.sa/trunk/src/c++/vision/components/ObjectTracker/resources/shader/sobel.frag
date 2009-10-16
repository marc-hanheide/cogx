
// handle to texture
uniform sampler2D frame;

// pixel offsets (to load from program)
uniform mat3 mOffsetX;
uniform mat3 mOffsetY;

// kernels for edge detection (to load from program)
uniform mat3 mSobelX;
uniform mat3 mSobelY;

// Threshold for removing noise
uniform float fThreshold;

const vec4 vNull = vec4(0.5,0.5,0.0,0.0);

vec4 sobel(){
	vec4 vTemp;
	float fTemp;
	float fGx, fGy;
	vec2 vG;
	float fGm;
	
	// convolute sobel kernel
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			vTemp = texture2D(frame, gl_TexCoord[0].st + vec2(mOffsetX[i][j], mOffsetY[i][j]));
			fTemp = length(vTemp);
			fGx += mSobelX[i][j] * fTemp;
			fGy += mSobelY[i][j] * fTemp;
		}
	}
	
	// normalized vector; range = [-1 ... 1]
	vG = vec2(fGx,fGy)/(mSobelY[0][0]+mSobelY[0][1]+mSobelY[0][2]);
	
	// magnitude of edge
	fGm = length(vG);
	
	// Threshold for removing background noise
	if(fGm < fThreshold)
		return vNull;
	
	// scale gradient to range = [0 ... 1]
	//vG = normalize(vG);
	vG = vG * 0.5 + 0.5;  
	return vec4(vG, fGm, 0.0);
}

void main(){
	gl_FragColor = sobel();
}

