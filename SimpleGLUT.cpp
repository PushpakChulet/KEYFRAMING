#include "stdafx.h"

// standard
#include <assert.h>
#include <math.h>

// glut
#include <GL/glut.h>

//================================
// global variables
//================================
// screen size
int g_screenWidth  = 0;
int g_screenHeight = 0;
GLfloat t = 0.0f; 
GLfloat dt = 0.01f;

// frame index
int g_frameIndex = 0;

// angle for rotation
int g_angle = 0;


GLint P = 0 ;
GLint N = 7;
GLfloat funcQT(GLfloat T[4], GLfloat mati[16], GLfloat controlpoints[4])
{
	GLfloat tempres[4] = { 0 };
	GLfloat Qt = 0;

	for (int i = 0; i < 4; i++) 
		for (int j = 0; j < 4; j++) 
			tempres[i] += T[j] * mati[4 * i + j];

	// Calcualte Qt
	for (int i = 0; i < 4; i++) 
		Qt += tempres[i] * controlpoints[i];
	
	return Qt;
}

GLfloat temp[16] = { 
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0
};

GLfloat Mcat[16] = { 
	-0.5f,	1.0f,	-0.5f,	0.0f,	  
	1.5f,	-2.5f,	0.0f,	1.0f,      
	-1.5f,	2.0f,	0.5f,	0.0f,      
	0.5f,	-0.5f,	0.0f,	0.0f 
};    

GLfloat MBspline[16] = { 
	-1.0/ 6.0,	3.0f / 6.0f,	-3.0f / 6.0f,	1.0f / 6.0f,
	3.0f / 6.0f,	-6.0f / 6.0f,	0.0f / 6.0f,	4.0f / 6.0f, 
	-3.0f / 6.0f,	3.0f / 6.0f,	3.0f / 6.0f,	1.0f / 6.0f, 
	1.0f / 6.0f,	0.0f / 6.0f,	0.0f / 6.0f,	0.0f / 6.0f 
};

GLfloat contorlpoints_quats[7][7] = {
	{ 1, 0, 0, 0, -5, 0, -5 },   
	{ 0, 1, 0, 0, -3, 3, -10 },  
	{ 0, 0, 1, 0, -1, 1, -15 },  
	{ 0, 0, 0, 1, 0, -5, -20 },  
	{ 0, 0, 1, 0, 1, 1, -15 },   
	{ 0, 1, 0, 0, 3, 3, -10 },  
	{ 1, 0, 0, 0, 5, 0, -5 } }; 

GLfloat contorlpoints_euler[7][6] = {
	{ 0.0, 0.0, 1.5, 0.0, 0, -5 },	
	{ 0.0, 1.0, 0.5, -3, 3, -10 },
	{ 0.0, -2.2, 2.5, -1, 0, -1 },
	{ 1.0, 0.0, 0.5, 0, -5, -20 },
	{ 0.8, 3.0, -0.5, 1, 1, -15 },
	{ 0.0, 1.0, -0.5, 3, 3, -10 },
	{ 0.0, -2.0, -0.5, 5, 0, -5 },
};

void quatToVect(GLfloat quat[7])
{
	GLfloat length = sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
	
	// base case
	if (length == 0)
		return;

	for (int i = 0; i < 4; i++) 
		quat[i] /= length;
}

void quatToRot(GLfloat quat[7], GLfloat rMat[16])
{
	GLfloat tempmat[4][4];
	for (int i = 0; i < 4; i++) 
		for (int j = 0; j < 4; j++) 
			tempmat[i][j] = quat[i] * quat[j];
	
	rMat[0] = 1.0f - 2.0f * tempmat[2][2] - 2.0f * tempmat[3][3];
	rMat[1] = 2.0f * tempmat[1][2] + 2.0f * tempmat[0][3];
	rMat[2] = 2.0f * tempmat[1][3] - 2.0f * tempmat[0][2];		   
	rMat[3] = 0.0f;					   
	rMat[4] = 2.0f * tempmat[1][2] - 2.0f * tempmat[0][3];
	rMat[5] = 1.0f - 2.0f * tempmat[1][1] - 2.0f * tempmat[3][3];
	rMat[6] = 2.0f * tempmat[2][3] + 2.0f * tempmat[0][1];
	rMat[7] = 0.0f;					  
	rMat[8] = 2.0f * tempmat[1][3] + 2.0f * tempmat[0][2];
	rMat[9] = 2.0f * tempmat[2][3] - 2.0f * tempmat[0][1];
	rMat[10] = 1.0f - 2.0f * tempmat[1][1] - 2.0f * tempmat[2][2];
	rMat[11] = 0.0f;					   
	rMat[12] = quat[4];				   
	rMat[13] = quat[5];			       
	rMat[14] = quat[6];			       
	rMat[15] = 1.0f;					   
}

GLfloat* eToQ(GLfloat euler_angles[6]) {
	GLfloat x = euler_angles[0] / 2;
	GLfloat y = euler_angles[1] / 2;
	GLfloat z = euler_angles[2] / 2;
	static GLfloat ans[7];
	ans[0] = cos(z) * cos(y) * cos(z) + sin(z) * sin(y) * sin(x);
	ans[1] = sin(z) * cos(y) * cos(z) - cos(z) * sin(y) * sin(x);
	ans[2] = cos(z) * sin(y) * cos(z) + sin(z) * cos(y) * sin(x);
	ans[3] = cos(z) * cos(y) * sin(z) - sin(z) * sin(y) * cos(x);
	ans[4] = euler_angles[3];
	ans[5] = euler_angles[4];
	ans[6] = euler_angles[5];

	return ans;
}


void toQuats(float quat[6][7], float spline[16]) {
	// Set up T matrix T = {t*t*t,t*t,t,1}
	float t_matrix[4] = { t * t * t, t * t, t, 1 };

	// interpolation orientation and position
	float quatval[7];

	for (int i = 0; i < 7; i++)
	{
		float M[4] = { 
			quat[P][i],
			quat[P + 1][i],
			quat[P + 2][i],
			quat[P + 3][i] 
		};

		quatval[i] = funcQT(t_matrix, spline, M);
	}

	quatToVect(quatval);
	quatToRot(quatval, temp);
}

void toEuler(float euler[7][6], float spline[16])
{
	// Set up T matrix T = {t*t*t,t*t,t,1}
	GLfloat T[4] = { t * t * t, t * t, t, 1 };

	// interpolation orientation and position
	GLfloat valEuler[7];

	for (int i = 0; i < 7; i++)
	{
		GLfloat M[4] = { 
			euler[P][i],
			euler[P + 1][i],
			euler[P + 2][i],
			euler[P + 3][i] 
		};

		valEuler[i] = funcQT(T, spline, M);
	}

	GLfloat* valQuat = eToQ(valEuler);
	quatToVect(valQuat);
	quatToRot(valQuat, temp);
}

//================================
// init
//================================
void init( void ) {
	// init something before main loop...
}

//================================
// update
//================================
void update( void ) {
	// do something before rendering...

	// rotation angle
	g_angle = ( g_angle + 5 ) % 360;
}
//================================
// render
//================================
void render( void ) {
	// clear buffer
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glClearDepth (1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	
	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[]	= { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT , LightAmbient );
	glLightfv(GL_LIGHT0, GL_DIFFUSE , LightDiffuse );
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[]	= { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[]	= { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[]	= { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[]	= { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se		= 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT	, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE	, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR	, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION	, material_Ke);
	glMaterialf (GL_FRONT, GL_SHININESS	, material_Se);

	// modelview matrix
	glMatrixMode( GL_MODELVIEW );
	
	// render objects
	glLoadMatrixf(temp);
	glutSolidTeapot(1.0);
	

	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
void keyboard( unsigned char key, int x, int y ) 
{
	//press and hold each key contineously to see the object move
	switch (key)
	{
	case 'a':
		toQuats(contorlpoints_quats, Mcat);
		break;
	case 's':
		toQuats(contorlpoints_quats,MBspline);
		break;
	case 'w':
		toEuler(contorlpoints_euler, Mcat);
		break;
	case 'd':
		toEuler(contorlpoints_euler,MBspline);
		break;
	default:
		break;
	}
}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape( int w, int h ) {
	// screen size
	g_screenWidth  = w;
	g_screenHeight = h;	
	
	// viewport
	glViewport( 0, 0, (GLsizei)w, (GLsizei)h );

	// projection matrix
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 2000.0);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer( int value ) 
{	
	glutPostRedisplay();

	t += dt;
	if (t >= 1)
	{
		t = 0;
		if (P < N - 4) {
			P++;
		}
		else {
			P = 0;
		}
	}
	// reset timer
	glutTimerFunc(16, timer, 0);
}

struct Resolution {
	int w;
	int h;

};

//================================
// main
//================================
int main( int argc, char** argv ) {
	// create opengL window
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
	glutInitWindowSize( 600, 600 ); 
	glutInitWindowPosition( 100, 100 );
	glutCreateWindow( argv[0] );

	// init
	init();
	
	// set callback functions
	glutDisplayFunc( render );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( keyboard );
	glutTimerFunc( 16, timer, 0 );
	
	// main loop
	glutMainLoop();

	return 0;
}
