varying vec4 vertex_model;
varying vec2 normal_edge;

uniform sampler2D tex_frame;

uniform float dwidth;
uniform float dheight;
uniform float max_distance;

const float pi = 3.141592654; 
const float pi2 = 1.570796327;  // pi2 = pi / 2;
const float pi4 = 0.785398163;  
const float dpi = 0.318309886;  // dpi = 1 / pi;

const vec4 white	= vec4(1.0, 1.0, 1.0, 1.0);
const vec4 red 		= vec4(1.0, 0.0, 0.0, 0.0);
const vec4 green 	= vec4(0.0, 1.0, 0.0, 0.0);
const vec4 blue 	= vec4(0.0, 0.0, 1.0, 0.0);
const vec4 black 	= vec4(0.0, 0.0, 0.0, 0.0);

void main(){
	vec4 texcoords_frame;
	vec4 color_frame;
	vec2 normal_frame;
	bool edge_found = false;
	float d;
	float fWeightAlpha = 0.0;
	float fWeightDistance = 0.0;
	float fWeight = 0.0;
	vec2 vWeightAlpha;
	vec2 en;
	float alpha;
	vec4 frag_color;
		
	// calculate texture coordinats for forward-mapping
	texcoords_frame = gl_ModelViewProjectionMatrix * vertex_model;
	texcoords_frame.x = (texcoords_frame.x / texcoords_frame.w + 1.0) * 0.5;
	texcoords_frame.y = (texcoords_frame.y / texcoords_frame.w + 1.0) * 0.5;
			
	//for(d=0.0; d<=max_distance && !edge_found; d+=1.0){
	for(d=0.0; d<=max_distance; d+=1.0){
	
		// positive side
		en.x = normal_edge.x * dwidth * d;
		en.y = normal_edge.y * dheight * d;
		color_frame = texture2D(tex_frame, texcoords_frame.xy + en);	// grab color normal to edge with distance d
		
		if(color_frame.z > 0.0){	// Check if pixel is part of an edge (Magnitude of sobel stored in z-value)
			
			// grab gradient vector of pixel
			normal_frame = color_frame.xy;
    		normal_frame = (normal_frame - 0.5) * 2.0;
    		normal_frame = normalize(normal_frame);
    		
    		// calculate angle between pixel and edge
			alpha = acos(normal_frame.x * normal_edge.x + normal_frame.y * normal_edge.y);
			if(alpha > pi2)
				alpha = pi - alpha;
    
    		// compute weight for edge-pixel
			edge_found = true;
			
			vWeightAlpha.x = 1.0 - alpha / pi2;
			if(vWeightAlpha.x > fWeightAlpha){
				fWeightAlpha = vWeightAlpha.x;
				fWeightDistance = 1.0 - d / max_distance;
			}
		}
		
		// negative side
		en.x = -en.x;
		en.y = -en.y;
		color_frame = texture2D(tex_frame, texcoords_frame.xy + en);	// grab color normal to edge with distance d
		
		if(color_frame.z > 0.0){	// Check if pixel is part of an edge (Magnitude of sobel stored in z-value)
			
			normal_frame = color_frame.xy;
    		normal_frame = (normal_frame - 0.5) * 2.0;
    		normal_frame = normalize(normal_frame);
    		
			alpha = acos(normal_frame.x * normal_edge.x + normal_frame.y * normal_edge.y);
    		if(alpha > pi2)
				alpha = pi - alpha;
				
			edge_found = true;
			
			vWeightAlpha.y = 1.0 - alpha / pi2;	
			if(vWeightAlpha.y > fWeightAlpha){
				fWeightAlpha = vWeightAlpha.y;
				fWeightDistance = 1.0 - d / max_distance;
			}
		}
	}
	
	if(edge_found){
		fWeight = 0.4*fWeightAlpha + 0.6*fWeightDistance;
		frag_color = vec4(fWeight,fWeight,fWeight,0.0);
	}else
		discard;		
		
	gl_FragColor = frag_color; //vec4(fWeightAlpha,fWeightAlpha,fWeightAlpha,0.0);
}

