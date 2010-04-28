varying vec4 vertex;

uniform sampler2D tex_model;

uniform mat4 modelviewprojection;

void main(){
	vec4 texcoords_model;
	vec4 color_model;
	vec4 frag_color;
	
	// calculate texture coordinats
	texcoords_model = modelviewprojection * vertex;
	texcoords_model.x = (texcoords_model.x / texcoords_model.w + 1.0) * 0.5;
	texcoords_model.y = (texcoords_model.y / texcoords_model.w + 1.0) * 0.5;
	color_model = texture2D(tex_model, texcoords_model.xy);
	
	// if not an edge ... discard
	if(color_model.z<0.1){
		discard;
	}else{
		frag_color = vec4(1.0,1.0,1.0,0.0);
	}
		
	// get texture color for frame and model
	gl_FragColor =  frag_color;

}



