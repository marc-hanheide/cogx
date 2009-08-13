varying vec4 vertex_model;
varying vec2 vEdgeDirection;

void main(){
	vertex_model = gl_Vertex;
    // Transform normal to projection plane
    vEdgeDirection = gl_ModelViewProjectionMatrix * vec4(gl_Normal,0.0);
    // Normalize projection of normal
    vEdgeDirection = normalize(vEdgeDirection);
    // Forward to fragment shader
    gl_Position = ftransform();
}
