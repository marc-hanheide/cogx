varying vec4 vertex;

uniform sampler2D texture;

uniform mat4 modelviewprojection;

uniform bool useTexCoords=true;

void main(){
	vec4 texcoords;
	
	if(useTexCoords){
		gl_FragColor = texture2D(texture, gl_TexCoord[0].xy);
	}else{
		// calculate texture coordinats
		texcoords = modelviewprojection * vertex;
		texcoords.x = (texcoords.x / texcoords.w + 1.0) * 0.5;
		texcoords.y = (texcoords.y / texcoords.w + 1.0) * 0.5;
		
		// get texture color for frame and model
		gl_FragColor =  texture2D(texture, texcoords.xy);
	}
}



