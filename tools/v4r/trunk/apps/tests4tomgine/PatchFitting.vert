uniform sampler2DShadow texNurb;
varying vec4 color;

void main(){
	vec4 img_pos = gl_ModelViewProjectionMatrix * gl_Vertex;
	vec3 tex_coord = img_pos.xyz / img_pos.w;
	tex_coord.x = (img_pos.x / img_pos.w + 1.0) * 0.5;
	tex_coord.y = (img_pos.y / img_pos.w + 1.0) * 0.5;
	tex_coord.z = (img_pos.z / img_pos.w + 1.0) * 0.5;

	float depth = shadow2D(texNurb, tex_coord);
	float error = img_pos.z - depth;

	if(error < 0.0)
		color = vec4(0.0, 1.0, 0.0, 0.0);
	else
		color = vec4(1.0, 0.0, 0.0, 0.0);

//	color = vec4(10.0*(depth-0.619), 0.0, 0.0, 0.0);

	gl_Position = img_pos;

}
