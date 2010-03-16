
# include "tgFrustum.h"

using namespace TomGine;

void tgFrustum::ExtractFrustum(){
	float   clip[16];
	float   t;
 
	/* Get the current PROJECTION matrix from OpenGL */
	glGetFloatv( GL_PROJECTION_MATRIX, m_proj );
 
	/* Get the current MODELVIEW matrix from OpenGL */
	glGetFloatv( GL_MODELVIEW_MATRIX, m_modl );
 
	/* Combine the two matrices (multiply projection by modelview) */
	clip[ 0] = m_modl[ 0] * m_proj[ 0] + m_modl[ 1] * m_proj[ 4] + m_modl[ 2] * m_proj[ 8] + m_modl[ 3] * m_proj[12];
	clip[ 1] = m_modl[ 0] * m_proj[ 1] + m_modl[ 1] * m_proj[ 5] + m_modl[ 2] * m_proj[ 9] + m_modl[ 3] * m_proj[13];
	clip[ 2] = m_modl[ 0] * m_proj[ 2] + m_modl[ 1] * m_proj[ 6] + m_modl[ 2] * m_proj[10] + m_modl[ 3] * m_proj[14];
	clip[ 3] = m_modl[ 0] * m_proj[ 3] + m_modl[ 1] * m_proj[ 7] + m_modl[ 2] * m_proj[11] + m_modl[ 3] * m_proj[15];
 
	clip[ 4] = m_modl[ 4] * m_proj[ 0] + m_modl[ 5] * m_proj[ 4] + m_modl[ 6] * m_proj[ 8] + m_modl[ 7] * m_proj[12];
	clip[ 5] = m_modl[ 4] * m_proj[ 1] + m_modl[ 5] * m_proj[ 5] + m_modl[ 6] * m_proj[ 9] + m_modl[ 7] * m_proj[13];
	clip[ 6] = m_modl[ 4] * m_proj[ 2] + m_modl[ 5] * m_proj[ 6] + m_modl[ 6] * m_proj[10] + m_modl[ 7] * m_proj[14];
	clip[ 7] = m_modl[ 4] * m_proj[ 3] + m_modl[ 5] * m_proj[ 7] + m_modl[ 6] * m_proj[11] + m_modl[ 7] * m_proj[15];
 
	clip[ 8] = m_modl[ 8] * m_proj[ 0] + m_modl[ 9] * m_proj[ 4] + m_modl[10] * m_proj[ 8] + m_modl[11] * m_proj[12];
	clip[ 9] = m_modl[ 8] * m_proj[ 1] + m_modl[ 9] * m_proj[ 5] + m_modl[10] * m_proj[ 9] + m_modl[11] * m_proj[13];
	clip[10] = m_modl[ 8] * m_proj[ 2] + m_modl[ 9] * m_proj[ 6] + m_modl[10] * m_proj[10] + m_modl[11] * m_proj[14];
	clip[11] = m_modl[ 8] * m_proj[ 3] + m_modl[ 9] * m_proj[ 7] + m_modl[10] * m_proj[11] + m_modl[11] * m_proj[15];
 
	clip[12] = m_modl[12] * m_proj[ 0] + m_modl[13] * m_proj[ 4] + m_modl[14] * m_proj[ 8] + m_modl[15] * m_proj[12];
	clip[13] = m_modl[12] * m_proj[ 1] + m_modl[13] * m_proj[ 5] + m_modl[14] * m_proj[ 9] + m_modl[15] * m_proj[13];
	clip[14] = m_modl[12] * m_proj[ 2] + m_modl[13] * m_proj[ 6] + m_modl[14] * m_proj[10] + m_modl[15] * m_proj[14];
	clip[15] = m_modl[12] * m_proj[ 3] + m_modl[13] * m_proj[ 7] + m_modl[14] * m_proj[11] + m_modl[15] * m_proj[15];
 
	/* Extract the numbers for the RIGHT plane */
	frustum[0][0] = clip[ 3] - clip[ 0];
	frustum[0][1] = clip[ 7] - clip[ 4];
	frustum[0][2] = clip[11] - clip[ 8];
	frustum[0][3] = clip[15] - clip[12];
 
	/* Normalize the result */
	t = sqrt( frustum[0][0] * frustum[0][0] + frustum[0][1] * frustum[0][1] + frustum[0][2] * frustum[0][2] );
	frustum[0][0] /= t;
	frustum[0][1] /= t;
	frustum[0][2] /= t;
	frustum[0][3] /= t;
 
	/* Extract the numbers for the LEFT plane */
	frustum[1][0] = clip[ 3] + clip[ 0];
	frustum[1][1] = clip[ 7] + clip[ 4];
	frustum[1][2] = clip[11] + clip[ 8];
	frustum[1][3] = clip[15] + clip[12];
 
	/* Normalize the result */
	t = sqrt( frustum[1][0] * frustum[1][0] + frustum[1][1] * frustum[1][1] + frustum[1][2] * frustum[1][2] );
	frustum[1][0] /= t;
	frustum[1][1] /= t;
	frustum[1][2] /= t;
	frustum[1][3] /= t;
 
	/* Extract the BOTTOM plane */
	frustum[2][0] = clip[ 3] + clip[ 1];
	frustum[2][1] = clip[ 7] + clip[ 5];
	frustum[2][2] = clip[11] + clip[ 9];
	frustum[2][3] = clip[15] + clip[13];
 
	/* Normalize the result */
	t = sqrt( frustum[2][0] * frustum[2][0] + frustum[2][1] * frustum[2][1] + frustum[2][2] * frustum[2][2] );
	frustum[2][0] /= t;
	frustum[2][1] /= t;
	frustum[2][2] /= t;
	frustum[2][3] /= t;
 
	/* Extract the TOP plane */
	frustum[3][0] = clip[ 3] - clip[ 1];
	frustum[3][1] = clip[ 7] - clip[ 5];
	frustum[3][2] = clip[11] - clip[ 9];
	frustum[3][3] = clip[15] - clip[13];
 
	/* Normalize the result */
	t = sqrt( frustum[3][0] * frustum[3][0] + frustum[3][1] * frustum[3][1] + frustum[3][2] * frustum[3][2] );
	frustum[3][0] /= t;
	frustum[3][1] /= t;
	frustum[3][2] /= t;
	frustum[3][3] /= t;
 
	/* Extract the FAR plane */
	frustum[4][0] = clip[ 3] - clip[ 2];
	frustum[4][1] = clip[ 7] - clip[ 6];
	frustum[4][2] = clip[11] - clip[10];
	frustum[4][3] = clip[15] - clip[14];
 
	/* Normalize the result */
	t = sqrt( frustum[4][0] * frustum[4][0] + frustum[4][1] * frustum[4][1] + frustum[4][2] * frustum[4][2] );
	frustum[4][0] /= t;
	frustum[4][1] /= t;
	frustum[4][2] /= t;
	frustum[4][3] /= t;
 
	/* Extract the NEAR plane */
	frustum[5][0] = clip[ 3] + clip[ 2];
	frustum[5][1] = clip[ 7] + clip[ 6];
	frustum[5][2] = clip[11] + clip[10];
	frustum[5][3] = clip[15] + clip[14];
 
	/* Normalize the result */
	t = sqrt( frustum[5][0] * frustum[5][0] + frustum[5][1] * frustum[5][1] + frustum[5][2] * frustum[5][2] );
	frustum[5][0] /= t;
	frustum[5][1] /= t;
	frustum[5][2] /= t;
	frustum[5][3] /= t;
}

bool tgFrustum::PointInFrustum( float x, float y, float z ){
	int p;
 
	for( p = 0; p < 6; p++ ){
		if( frustum[p][0] * x + frustum[p][1] * y + frustum[p][2] * z + frustum[p][3] <= 0 )
			return false;
	}
			
	return true;
}

bool tgFrustum::SphereInFrustum( float x, float y, float z, float radius ){
	int p;
 
	for( p = 0; p < 6; p++ ){
		if( frustum[p][0] * x + frustum[p][1] * y + frustum[p][2] * z + frustum[p][3] <= -radius )
			return false;
	}
	return true;
}

void tgFrustum::DrawFrustum(){
 
	// Get near and far from the Projection matrix.
	const double near = m_proj[11] / (m_proj[10] - 1.0);
	const double far = m_proj[11] / (1.0 + m_proj[10]);
 
	// Get the sides of the near plane.
	const double nLeft = near * (m_proj[2] - 1.0) / m_proj[0];
	const double nRight = near * (1.0 + m_proj[2]) / m_proj[0];
	const double nTop = near * (1.0 + m_proj[6]) / m_proj[5];
	const double nBottom = near * (m_proj[6] - 1.0) / m_proj[5];
 
	// Get the sides of the far plane.
	const double fLeft = far * (m_proj[2] - 1.0) / m_proj[0];
	const double fRight = far * (1.0 + m_proj[2]) / m_proj[0];
	const double fTop = far * (1.0 + m_proj[6]) / m_proj[5];
	const double fBottom = far * (m_proj[6] - 1.0) / m_proj[5];
 
	/*
	 0	glVertex3f(0.0f, 0.0f, 0.0f);
	 1	glVertex3f(nLeft, nBottom, -near);
	 2	glVertex3f(nRight, nBottom, -near);
	 3	glVertex3f(nRight, nTop, -near);
	 4	glVertex3f(nLeft, nTop, -near);
	 5	glVertex3f(fLeft, fBottom, -far);
	 6	glVertex3f(fRight, fBottom, -far);
	 7	glVertex3f(fRight, fTop, -far);
	 8	glVertex3f(fLeft, fTop, -far);
	 */
 
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
		
					// TODO - Update: You need to invert the mv before multiplying it with the current mv!
	
		glMultMatrixf(m_modl);
	
		glLineWidth(2);
		glBegin(GL_LINES);
	
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(fLeft, fBottom, -far);
		
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(fRight, fBottom, -far);
		
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(fRight, fTop, -far);
		
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(fLeft, fTop, -far);
		
			//far
			glVertex3f(fLeft, fBottom, -far);
			glVertex3f(fRight, fBottom, -far);
		
			glVertex3f(fRight, fTop, -far);
			glVertex3f(fLeft, fTop, -far);
		
			glVertex3f(fRight, fTop, -far);
			glVertex3f(fRight, fBottom, -far);
		
			glVertex3f(fLeft, fTop, -far);
			glVertex3f(fLeft, fBottom, -far);
		
			//near
			glVertex3f(nLeft, nBottom, -near);
			glVertex3f(nRight, nBottom, -near);
		
			glVertex3f(nRight, nTop, -near);
			glVertex3f(nLeft, nTop, -near);
		
			glVertex3f(nLeft, nTop, -near);
			glVertex3f(nLeft, nBottom, -near);
		
			glVertex3f(nRight, nTop, -near);
			glVertex3f(nRight, nBottom, -near);
	
		glEnd();
		glLineWidth(1);
	glPopMatrix();

}