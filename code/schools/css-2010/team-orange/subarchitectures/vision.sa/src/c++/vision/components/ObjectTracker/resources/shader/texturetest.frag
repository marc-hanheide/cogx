varying vec4 vertex_model;

uniform sampler2D tex_frame;
uniform sampler2D tex_model;

uniform mat4 modelviewprojection; // -matrix of previouse tracked position

uniform float fTol; // tolerance for angle comparison
uniform bool compare;
uniform bool analyze;

uniform vec4 drawcolor;

const float inf = 100000.0;
const float dpi = 0.318309886;  // dpi = 1 / pi;

const vec4 white	= vec4(1.0, 1.0, 1.0, 1.0);
const vec4 red 		= vec4(1.0, 0.0, 0.0, 0.0);
const vec4 green 	= vec4(0.0, 1.0, 0.0, 0.0);
const vec4 blue 	= vec4(0.0, 0.0, 1.0, 0.0);
const vec4 black 	= vec4(0.0, 0.0, 0.0, 0.0);

void main(){
	vec4 texcoords_model;
	vec4 texcoords_frame;
	vec4 color_frame;
	vec4 color_model;
	vec2 gradient_frame;
	vec2 gradient_model;
	float alpha;
	
	// calculate texture coordinats for forward-mapping
	texcoords_model = modelviewprojection * vertex_model;
	texcoords_model.x = (texcoords_model.x / texcoords_model.w + 1.0) * 0.5;
	texcoords_model.y = (texcoords_model.y / texcoords_model.w + 1.0) * 0.5;
	
	// calculate texture coordinats for forward-mapping
	texcoords_frame = gl_ModelViewProjectionMatrix * vertex_model;
	texcoords_frame.x = (texcoords_frame.x / texcoords_frame.w + 1.0) * 0.5;
	texcoords_frame.y = (texcoords_frame.y / texcoords_frame.w + 1.0) * 0.5;	
	
	// get texture color for frame and model
	color_model = texture2D(tex_model, texcoords_model.xy);
	color_frame = texture2D(tex_frame, texcoords_frame.xy);
	
	//gl_FragColor = color_frame;
	//return;
	
	// pixel of model is not an edge pixel
	if( color_model.z<0.01 ){
		discard;
		//gl_FragDepth = inf;
		//return;
	}
			
	if( !compare ){
		gl_FragColor = drawcolor;
			
		return;
	}
	
	// pixel of frame is not an edge pixel
	if( color_frame.z<0.01 ){
		//discard;
		gl_FragColor = black;
		if(!analyze)
			discard;
		return;
	}
	
	// disregarding edge direction
	//gl_FragColor = red;
	//return; 
	
	// calculate edge gradients of frame and model
	gradient_frame = (color_frame.xy - 0.5) * 2.0;
	gradient_model = (color_model.xy - 0.5) * 2.0;
	
	gradient_frame = normalize(gradient_frame);
	gradient_model = normalize(gradient_model);
	
	
	// calculate edge angle between frame and model; range(alpha) = [0...180]
	alpha = acos(gradient_frame.x * gradient_model.x + gradient_frame.y * gradient_model.y) * 180.0 * dpi;
	
	// compare edges
	if( (alpha < (0.0 + fTol)) || (alpha > (180.0 - fTol)) ){	// more robust to rotations
	//if( (alpha < (0.0 + fTol)) ){								// more robust to background clutter
		gl_FragColor = red;
		return;
	}else{
		gl_FragColor = blue;
		if(!analyze)
			discard;
		return;
	}
}



