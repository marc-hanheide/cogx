
uniform sampler2D frame;

uniform mat3 mOffsetX;
uniform mat3 mOffsetY;

void main(){
	vec4 vColor, vNeighbour;
    vec4 vMaxNeighbour = vec4(0.0, 0.0, 0.0, 0.0);
    
    // get gradient and magnitude of pixel stored in color
    vColor = texture2D(frame, gl_TexCoord[0]);
    
    // if pixel is an edge pixel, do nothing
    if(vColor.z < 0.01){    
		// parse through neighbouring pixels
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				vNeighbour = texture2D(frame, gl_TexCoord[0].st + vec2(mOffsetX[i][j], mOffsetY[i][j]));
				// if magnitude of adjacent pixel is bigger than a threshold
				// set magnitude of this pixel to that of the adjacent pixel considering distance
				if(vNeighbour.z > vMaxNeighbour.z){
					vMaxNeighbour = vNeighbour;
					vColor = vMaxNeighbour;
				}else{
					//vColor = vec4(0.0,0.0,0.0,0.0);
				}
			}
		}
	}
    
    gl_FragColor = vColor;
}
