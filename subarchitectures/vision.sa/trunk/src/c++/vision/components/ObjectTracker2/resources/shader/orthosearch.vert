varying vec4 vertex_model;
varying vec2 normal_edge;

void main(){
	vec2 edge;
	
	vertex_model = gl_Vertex;
  // Transform normal to projection plane
  edge = gl_ModelViewProjectionMatrix * vec4(gl_Normal,0.0);
  // Normalize projection of normal
  edge = normalize(edge);
  normal_edge.y = edge.x;
  normal_edge.x = -edge.y;
  
  // Forward to fragment shader
  gl_Position = ftransform();
}
