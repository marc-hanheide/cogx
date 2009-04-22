
// handle to texture
uniform sampler2D frame;

// pixel offsets (to load from program)
uniform mat3 mOffsetX;
uniform mat3 mOffsetY;

// threshold for compare statement
uniform float fThreshold = 0.0;

const vec4 vNull = vec4(0.5,0.5,0.0,0.0);
const float pi = 3.141592;

vec2 getOffset(vec2 vG){
    ivec2 vNb = ivec2(0,0);
    vec2 offset = vec2(0.0,0.0);
    
    // find pixel to gradient direction
    if(vG.x > vG.y){
        offset = vec2(mOffsetX[1][2],mOffsetY[1][2]);
    }else{
        offset = vec2(mOffsetX[0][1],mOffsetY[0][1]);
    }
    return offset;
}

vec4 thinning(){
    vec2 vG;
    vec2 vN;
    vec2 offset;
    float ft = fThreshold;
    float f_len_1, f_len_2;
    
    // get edge gradient from texture
    vG = texture2D(frame, gl_TexCoord[0].st).xy;
    vG = (vG - 0.5) * 2.0;  // scale to range [-1 ... 1]
    //vG = normalize(vG);
    
    // if pixel is not part of an edge then do nothing
    if(abs(vG.x) < 0.01 && abs(vG.y) < 0.01)
        return vNull;
    
    // get offset in texture coordinates for neighbouring pixel
    // in edge gradient direction
    offset = getOffset(vG);
    
    // get color of neighbouring pixel
    vN = texture2D(frame, gl_TexCoord[0].st + offset ).xy;
    vN = (vN - 0.5) * 2.0;  // scale to  [-1 ... 1]
    // remove this pixel if neighbouring pixel is stronger
    f_len_1 = length(vN);
    if(length(vG)+ft < f_len_1) 
        return vNull;
    
    // Compare again for negative gradient direction 
    vN = texture2D(frame, gl_TexCoord[0].st - offset ).xy;
    vN = (vN - 0.5) * 2.0;
    
    // remove this pixel if neighbouring pixel is stronger
    f_len_2 = length(vN);
    if(length(vG)+ft < f_len_2) 
        return vNull;
        
    
    // remove this pixel if neighbouring pixels are no edges
    if( (f_len_1 < 0.01) && (f_len_2 < 0.01) )
    	return vNull;
    
    
    // set magnitude to 1
    vG = normalize(vG);
    
    // scale to range [0 ... 1]
    vG = vG * 0.5 + 0.5;
    // return gradient (xy) and magnitude (z) and safe it as color
    return vec4(vG,1.0,0.0);
}

void main(){
    gl_FragColor = thinning();
}
