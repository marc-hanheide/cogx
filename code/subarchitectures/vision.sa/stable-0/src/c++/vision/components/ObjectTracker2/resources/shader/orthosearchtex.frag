varying vec4 vertex;

uniform sampler2D tex_frame;
uniform sampler2D tex_model;

uniform float dwidth;
uniform float dheight;
uniform float max_distance;
uniform float alpha_tol;

uniform mat4 modelviewprojection;

const float pi = 3.141592654;
const float pi2 = 1.570796327;  // pi2 = pi / 2;
const float dpi = 0.318309886;  // dpi = 1 / pi;


void main(){
	vec4 texcoords_model;
	vec4 texcoords_frame;
	vec4 color_model;
	vec4 color_frame;
	vec4 frag_color;
	vec2 normal_model;
	vec2 normal_frame;
	float d;
	vec2 en;
	float alpha;
	bool edge_found = false;
	float fWeightAlpha = 0.0;
	float fWeightDistance = 0.0;
	float fWeight = 0.0;
	vec2 vWeightAlpha;
	
	
	
	// calculate texture coordinats
	texcoords_model = modelviewprojection * vertex;
	texcoords_model.x = (texcoords_model.x / texcoords_model.w + 1.0) * 0.5;
	texcoords_model.y = (texcoords_model.y / texcoords_model.w + 1.0) * 0.5;
	color_model = texture2D(tex_model, texcoords_model.xy);
	
	// if not an edge ... discard
	if(color_model.z<0.1){
		discard;
	}
		
	// convert to range [-1 ... 1]
	normal_model = color_model.xy;
	normal_model = (normal_model - 0.5) * 2.0;
	normal_model = normalize(normal_model);
	
	// calculate texture coordinats for forward-mapping
	texcoords_frame = gl_ModelViewProjectionMatrix * vertex;
	texcoords_frame.x = (texcoords_frame.x / texcoords_frame.w + 1.0) * 0.5;
	texcoords_frame.y = (texcoords_frame.y / texcoords_frame.w + 1.0) * 0.5;
	
	
	for(d=0.0; d<=max_distance && !edge_found; d+=1.0){
	
		// positive side
		en.x = normal_model.x * dwidth * d;
		en.y = normal_model.y * dheight * d;
		color_frame = texture2D(tex_frame, texcoords_frame.xy + en);	// grab color normal to edge with distance d
		
		if(color_frame.z > 0.1){	// Check if pixel is part of an edge (Magnitude of sobel stored in z-value)
			
			// grab gradient vector of pixel
			normal_frame = color_frame.xy;
    		normal_frame = (normal_frame - 0.5) * 2.0;
    		normal_frame = normalize(normal_frame);
    		
    		// calculate angle between pixel and edge
			alpha = acos(normal_frame.x * normal_model.x + normal_frame.y * normal_model.y);
			if(alpha > pi2)
				alpha = pi - alpha;
    
    		if(alpha < alpha_tol){
				edge_found = true;
				fWeightDistance = 1.0 - d/max_distance;
				fWeightAlpha = 1.0 - alpha/pi2;
			}
		}
		
		// negative side
		en.x = -en.x;
		en.y = -en.y;
		color_frame = texture2D(tex_frame, texcoords_frame.xy + en);	// grab color normal to edge with distance d
		
		if(color_frame.z > 0.1){	// Check if pixel is part of an edge (Magnitude of sobel stored in z-value)
			
			normal_frame = color_frame.xy;
    		normal_frame = (normal_frame - 0.5) * 2.0;
    		normal_frame = normalize(normal_frame);
    		
			alpha = acos(normal_frame.x * normal_model.x + normal_frame.y * normal_model.y);
    		if(alpha > pi2)
				alpha = pi - alpha;
			
			if(alpha < alpha_tol){
				edge_found = true;
				fWeightDistance = 1.0 - d/max_distance;
				fWeightAlpha = 1.0 - alpha/pi2;
			}
		}
	}
	
	if(edge_found){
		fWeight = 0.0*fWeightAlpha + 1.0*fWeightDistance;
		frag_color = vec4(fWeight,fWeight,fWeight,0.0);
	}else
		discard;
	
	// get texture color for frame and model
	gl_FragColor =  frag_color;

}



