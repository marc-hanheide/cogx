
uniform sampler2D tex1;
uniform sampler2D tex2;

void main(){
	vec4 vColor1 = texture2D(tex1, gl_TexCoord[0].st);
	vec4 vColor2 = texture2D(tex2, gl_TexCoord[0].st);
	
	vec4 vColor = vec4(	vColor1.r*vColor2.r,
						vColor1.g*vColor2.g,
						vColor1.b*vColor2.b,
						vColor1.a*vColor2.a );

	gl_FragColor = vColor;
}
