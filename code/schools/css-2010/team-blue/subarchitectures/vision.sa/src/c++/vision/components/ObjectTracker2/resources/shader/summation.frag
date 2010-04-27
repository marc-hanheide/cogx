/*
*	high precision summation shader
*	converts image to gray scale image (green, blue and alpha channel are used for raising precision)
*	range of each channel in unsigned byte r: [0..100] bga: [0..99]
*	each channel represents 2 positions of the value
*/
uniform sampler2D image;

uniform float d;		// half step width to pixel of higher LOD
uniform int first;		// determine if first run of texture (because of confersion)

const float b = 0.003921569; // b = 1/255
  						
void main(){
	vec4 p[4];
	vec4 v,r;
	vec4 vSum;
	vec4 of; 	// overflow
	
	float m = gl_TexCoord[0].x;
	float n = gl_TexCoord[0].y;
	
	if(first == 2){
		m = m+0.5;
		n = n+0.5;
	}
	
	//gl_FragColor = vec4(m,n,0.0,0.0);	// for debugging
	//return;
		
	if(first == 1){
		// if first run convert pixels to grayscale using all 4 channels for higher precision
		p[0] = texture2D(image, vec2(m-d, n-d));
		p[1] = texture2D(image, vec2(m-d, n+d));
		p[2] = texture2D(image, vec2(m+d, n-d));
		p[3] = texture2D(image, vec2(m+d, n+d));
		/*
		p[0] = vec4(100.0*p[0].x*b, 0.0, 0.0, 0.0);
		p[1] = vec4(100.0*p[1].x*b, 0.0, 0.0, 0.0);
		p[2] = vec4(100.0*p[2].x*b, 0.0, 0.0, 0.0);
		p[3] = vec4(100.0*p[3].x*b, 0.0, 0.0, 0.0);
		*/
		// convert to gray level and store in internal format in RED chanel
		p[0] = vec4(100.0*b*(0.299*p[0].r+0.587*p[0].g+0.114*p[0].b), 0.0, 0.0, 0.0);
		p[1] = vec4(100.0*b*(0.299*p[1].r+0.587*p[1].g+0.114*p[1].b), 0.0, 0.0, 0.0);
		p[2] = vec4(100.0*b*(0.299*p[2].r+0.587*p[2].g+0.114*p[2].b), 0.0, 0.0, 0.0);
		p[3] = vec4(100.0*b*(0.299*p[3].r+0.587*p[3].g+0.114*p[3].b), 0.0, 0.0, 0.0);
	}else{
		p[0] = texture2D(image, vec2(m-d, n-d));
		p[1] = texture2D(image, vec2(m-d, n+d));
		p[2] = texture2D(image, vec2(m+d, n-d));
		p[3] = texture2D(image, vec2(m+d, n+d));
	}
	
	// sum and devide
	v.x = (p[0].x+p[1].x+p[2].x+p[3].x)*63.75; 	// average of pixels 63.75 = 255/4
	vSum.x = floor(v.x);						// cut away remainder
	r.x = v.x - vSum.x;							// store remainder for next precision level
	vSum.x = vSum.x;							// convert from [0..100] to [0..1] 
	
	v.y = (p[0].y+p[1].y+p[2].y+p[3].y)*63.75;
	vSum.y = floor(v.y);
	r.y = v.y - vSum.y;
	vSum.y = (vSum.y + r.x*100.0); // add remainder of previouse precision level
	
	v.z = (p[0].z+p[1].z+p[2].z+p[3].z)*63.75;
	vSum.z = floor(v.z);
	r.z = v.z - vSum.z;
	vSum.z = (vSum.z + r.y*100.0);
	
	v.w = (p[0].w+p[1].w+p[2].w+p[3].w)*63.75;
	vSum.w = floor(v.w);
	r.w = v.w - vSum.w;
	vSum.w = (vSum.w + floor(r.z*100.0));
	
	// round highest precision level
	if(r.w >= 0.5)
		vSum.w = vSum.w + 1.0;
	
	// forward overflow to lower precision levels
	if(vSum.w >= 100.0){
		of.z = floor(vSum.w/100.0);		// determine overflow
		vSum.w = vSum.w - 100.0*of.z;	// subtract overflow from current precision level
		vSum.z = vSum.z + of.z;			// add overflow to lower precision level
	}
	if(vSum.z >= 100.0){
		of.y = floor(vSum.z/100.0);
		vSum.z = vSum.z - 100.0*of.y;
		vSum.y = vSum.y + of.y;
	}
	if(vSum.y >= 100.0){
		of.x = floor(vSum.y/100.0);
		vSum.y = vSum.y - 100.0*of.x;
		vSum.x = vSum.x + of.x;
	}
	
	// Check for error (average cannot be above 1)
	if(vSum.x > 100.0){
		gl_FragColor = vec4(1.0,0.0,0.0,0.0);
		return;
	}
	
	gl_FragColor = vSum*b;

}

