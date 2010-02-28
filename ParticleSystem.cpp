/*
 *  Lab exercise 5.0 
 *  CSc 474, Computer Graphics
 *  Copyright 2000-2002 Chris Buckalew
 *
 * Simple Euler Particle System
 *
 * by Peter Palombi and Chris Buckalew
 *
 * This simple particle system shoots particles up and to the
 * right.  Make the following modifications:
 *
 * [x] 1) Reset the particles to recycle when they fall below y=0
 *
 * [x] 2) Randomize the initial velocity so that you get a "spray" 
 *    effect   
 *
 * [x] 3) Add a wind force which is constant - like gravity only
 *    don't multiply by the mass
 *
 * [x] 4) Add an air resistance force that operates in the opposite
 *    direction to velocity, and is proportional to the square
 *    of the velocity
 *
 * [x] 5) Play with the drag and mass constants to get a nice 
 *    fountain effect
 *
 * [x] 6) Now change the wind force so that it increases with height -
 *    the wind is stronger 10 units up than it is at 1 unit up.
 *
 * [ ] 7) Now make the fountain into a firework fountain by adding 
 *    airbursts when the particles have been traveling a certain
 *    length of time.  Replace the particle with multiple particles
 *    having different initial velocities.  Make them different colors,
 *    too.
 *
 *------------------------------------------------------------------*/


#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <gl/gl.h>
#include <gl/glu.h>
#include <gl/glut.h>

// some function prototypes
void display(void);
void sumForces(void);
void EulerIntegrate(void);
void normalize(float[3]);
void normCrossProd(float[3], float[3], float[3]);

// no provision to move viewpoint in this code,
//   but here are the transforms anyway
// initial viewer position
static GLdouble viewer[] = {0.0, 0.0, 10.0};
// initial viewer angle
static GLfloat theta[] = {0.0, 0.0, 0.0};

// animation variables
static int frame = 0;
static int startFrame = 0;
static int endFrame = 100;
static int increment = 1;

const int NUM_PARTICLES = 10;
const float PARTICLE_FLOOR = 0.0;
const float GRAVITY = 9.8;
const float DRAG = 0.1;
const float TIMESTEP = 0.1;
const float WIND_HEIGHT_FACTOR = 0.1;
const float WIND = 2.0;
const float MASS_MIN = 3.0;
const float MASS_MAX = 3.0;
const float INIT_VEL_MIN[3] = {-2.0,  12.0, 0.0};
const float INIT_VEL_MAX[3] = { 2.0, 15.0, 0.0};

float mass[NUM_PARTICLES];
float pos[NUM_PARTICLES][3];
float vel[NUM_PARTICLES][3];
float acc[NUM_PARTICLES][3];
float force[NUM_PARTICLES][3];
//float lifetime[NUM_PARTICLES];

//---------------------------------------------------------
//   Set up the view

void setUpView() {
   // this code initializes the viewing transform
   glLoadIdentity();

   // moves viewer along coordinate axes
   gluLookAt(viewer[0], viewer[1], viewer[2], 
             0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

   // move the view back some relative to viewer[] position
   glTranslatef(0.0f,0.0f, 0.0f);

   // rotates view
   glRotatef(theta[0], 1.0, 0.0, 0.0);
   glRotatef(theta[1], 0.0, 1.0, 0.0);
   glRotatef(theta[2], 0.0, 0.0, 1.0);

   return;
}

//--------------------------------------------------------
//  Set up the objects

void drawParticles() {
   // draw all the particles

   for (int i=0; i<NUM_PARTICLES; i++) {
      // for each particle

      // save the transformation state
      glPushMatrix();

      // this translation will be used to animate the particle
      glTranslatef(pos[i][0], pos[i][1], pos[i][2]);	 

      // each particle is just a glPoint
      glBegin(GL_POINTS);
         glColor3f(0.4, 0.9, 0.4);
         glVertex3f(0.0, 0.0, 0.0);
      glEnd();

     // recover the transform state
      glPopMatrix();
   }

   return;
}

//-----------------------------------------------------------
//  Callback functions

void reshapeCallback(int w, int h) {
   // from Angel, p.562

   glViewport(0,0,w,h);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   if (w < h) {
      glFrustum(-2.0, 2.0, -2.0*(GLfloat) h / (GLfloat) w,
                2.0*(GLfloat) h / (GLfloat) w, 2.0, 20.0);
   }
   else {
      glFrustum(-2.0, 2.0, -2.0*(GLfloat) w / (GLfloat) h,
                2.0*(GLfloat) w / (GLfloat) h, 2.0, 20.0);
   }

   glMatrixMode(GL_MODELVIEW);
}


void timeStep(int step) {
   // animation code goes here
   
   // This function is called for each frame of animation
   double t = (double) (frame - startFrame) / (endFrame - startFrame);
   
   // do the particle system stuff
   sumForces();
   EulerIntegrate();
   
   if (frame == endFrame) increment = -1;
   else if (frame == startFrame) increment = 1;
    
   frame = frame + increment;

   display();

   glutTimerFunc(50,timeStep, 0);

}

//---------------------------------------------------------
//  Particle system routines



float randFloat() {
   return ((float)rand()) / RAND_MAX;
}

float randFloat(float min, float max) {
   float scale = max - min;
   return min + randFloat() * scale;
}

void initializeParticle(int index) {
   int i = index;
   mass[i] = randFloat(MASS_MIN, MASS_MAX);

   pos[i][0] = 0.0;
   pos[i][1] = 0.0;
   pos[i][2] = 0.0;

   vel[i][0] = randFloat(INIT_VEL_MIN[0], INIT_VEL_MAX[0]);
   vel[i][1] = randFloat(INIT_VEL_MIN[1], INIT_VEL_MAX[1]);
   vel[i][2] = 0.0;
   
   acc[i][0] = 0.0;
   acc[i][1] = 0.0;
   acc[i][2] = 0.0;
   
   force[i][0] = 0.0;
   force[i][1] = 0.0;
   force[i][2] = 0.0;
}

void initializeParticleDataStructure() {
   // initial values for all the particles

   for (int i=0; i<NUM_PARTICLES; i++) {
      initializeParticle(i);
   }
}

void sumForces() {
   // for this timestep, accumulate all the forces that
   //   act on each particle

   for (int i=0; i<NUM_PARTICLES; i++) {

      // ZERO ALL FORCES
      force[i][0] = force[i][1] = force[i][2] = 0.0;

      // GRAVITY
      force[i][1] += -GRAVITY*mass[i];

      // WIND
      force[i][0] += pos[i][1] * WIND_HEIGHT_FACTOR * WIND;

      // AIR RESISTANCE
      if(vel[i][0] > 0) {
         force[i][0] -= DRAG * vel[i][0] * vel[i][0];
      } else {
         force[i][0] += DRAG * vel[i][0] * vel[i][0];
      }
      if(vel[i][1] > 0) {
         force[i][1] -= DRAG * vel[i][1] * vel[i][1];
      } else {
         force[i][1] += DRAG * vel[i][1] * vel[i][1];
      }
      
   }
}
  
void EulerIntegrate() {
   // for each particle, compute the new velocity
   //   and position

   for (int i = 0; i < NUM_PARTICLES; i++) {
      
  
      // CALCULATE NEW ACCEL
      acc[i][0] = force[i][0] / mass[i];
      acc[i][1] = force[i][1] / mass[i];
      acc[i][2] = force[i][2] / mass[i];

      // CALCULATE NEW POS
      pos[i][0] += vel[i][0] * TIMESTEP +
                        0.5 * acc[i][0] * TIMESTEP * TIMESTEP;
      pos[i][1] += vel[i][1] * TIMESTEP +
                        0.5 * acc[i][1] * TIMESTEP * TIMESTEP;
      pos[i][2] += vel[i][2] * TIMESTEP +
                        0.5 * acc[i][2] * TIMESTEP * TIMESTEP;
   
      // CALCULATE NEW VEL
      vel[i][0] += acc[i][0] * TIMESTEP;
      vel[i][1] += acc[i][1] * TIMESTEP;
      vel[i][2] += acc[i][2] * TIMESTEP;

      // recycle particle if it has fallen below y=0
      if(pos[i][1] < PARTICLE_FLOOR) {
         initializeParticle(i);
      }
      
   }
}
  
//---------------------------------------------------------
//  Main routines

void display (void) {
   // this code executes whenever the window is redrawn (when opened,
   //   moved, resized, etc.
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   // set the viewing transform
   setUpView();

   // start drawing objects
   drawParticles();

   glutSwapBuffers();
}

// create a double buffered 500x500 pixel color window
int main(int argc, char* argv[]) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Particle System: Lab 5");
   initializeParticleDataStructure();

   // this sets the size of a point in pixels
   glPointSize(5.0);

	glEnable(GL_DEPTH_TEST);
	glutDisplayFunc(display);
   glutReshapeFunc(reshapeCallback);
	glutTimerFunc(50,timeStep, 0);  // 50 millisecond callback
	glutMainLoop();
	return 0;
}

//---------------------------------------------------------
//  Utility functions

void normalize(float v[3]) {
   // normalize v[] and return the result in v[]
   // from OpenGL Programming Guide, p. 58
   GLfloat d = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
   if (d == 0.0) {
      printf("zero length vector");
      return;
   }
   v[0] = v[0]/d; v[1] = v[1]/d; v[2] = v[2]/d;
}

void normCrossProd(float v1[3], float v2[3], float out[3]) {
   // cross v1[] and v2[] and return the result in out[]
   // from OpenGL Programming Guide, p. 58
   out[0] = v1[1]*v2[2] - v1[2]*v2[1];
   out[1] = v1[2]*v2[0] - v1[0]*v2[2];
   out[2] = v1[0]*v2[1] - v1[1]*v2[0];
   normalize(out);
}

