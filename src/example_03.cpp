#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <deque>
#include <vector>
#include <string>

//include header file for glfw library so that we can use OpenGL
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "bezpatch.h"
#include "bezfile.h"
#include "vec3.h"

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#define PI 3.14159265 // Should be used from mathlib

using namespace std;

/*
For UC Berkeley's CS184 Fall 2016 course, assignment 3 (Bezier surfaces)
*/

//****************************************************
// Global Variables
//****************************************************
GLfloat translation[3] = {0.0f, 0.0f, 0.0f};
GLfloat rotation[3] = {0.0f, 0.0f, 0.0f};
bool wireframe_mode = false;
bool flat_shading = false;
bool auto_strech = false;
int Width_global = 400;
int Height_global = 400;
int Z_buffer_bit_depth = 128;
float zoom = 0.5f;
vector<BezPatch> patches;
float subdivisionParam;
bool adaptive_subdivision = false;
Vec3 viewboxPos;
float viewboxSize;

inline float sqr(float x) { return x*x; }


//****************************************************
// Simple init function
//****************************************************
void initializeRendering()
{
    glfwInit();
}

//****************************************************
// Keyboard inputs. Add things to match the spec! 
//****************************************************
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    switch (key) {
            
        case GLFW_KEY_ESCAPE:
        case GLFW_KEY_Q:
          glfwSetWindowShouldClose(window, GLFW_TRUE);
          break;
        case GLFW_KEY_W:
          if (action == GLFW_PRESS) {
            if (wireframe_mode)
              glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
            else
              glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
            wireframe_mode = !wireframe_mode;
          }
          break;
        case GLFW_KEY_S:
          if (action == GLFW_PRESS) {
            if (flat_shading)
              glShadeModel (GL_SMOOTH);
            else
              glShadeModel (GL_FLAT);
            flat_shading = !flat_shading;
          }
          break;
        case GLFW_KEY_LEFT :
          if (action) {
            if (mods == GLFW_MOD_SHIFT) {
              translation[0] += 0.001f * Width_global;
            } else {
              rotation[0] -= 2;
            }
          }
          break;
        case GLFW_KEY_RIGHT:
          if (action) {
            if (mods == GLFW_MOD_SHIFT) {
              translation[0] -= 0.001f * Width_global;
            } else {
              rotation[0] += 2;
            }
          }
          break;
        case GLFW_KEY_UP   :
          if (action) {
            if (mods == GLFW_MOD_SHIFT) {
              translation[1] -= 0.001f * Height_global;
            } else {
              rotation[1] -= 2;
            }
          }
          break;
        case GLFW_KEY_DOWN :
          if (action) {
            if (mods == GLFW_MOD_SHIFT) {
              translation[1] += 0.001f * Height_global;
            } else {
              rotation[1] += 2;
            }
          }
          break;
        case GLFW_KEY_MINUS :
          if (action) {
            zoom /= .8f;
          }
          break;
        case GLFW_KEY_EQUAL :
          if (action) {
            zoom *= .8f;
          }
          break; 
        case GLFW_KEY_F:
          if (action && mods == GLFW_MOD_SHIFT) auto_strech = !auto_strech; break;
        default: break;
    }
    
}

void glLocalGeo (const LocalGeo& local) {
  glNormal3f(local.normal.x, local.normal.y, local.normal.z);
  glVertex3f(local.pos.x, local.pos.y, local.pos.z);
}

void renderBezierUniform (const BezPatch& patch, float step) {
  int numdiv = 1.00001 / step;
  LocalGeo** surface = new LocalGeo*[numdiv+1];
  for (int i = 0; i < numdiv+1; i++) {
    surface[i] = new LocalGeo[numdiv+1];
  }

  for (int i = 0; i < numdiv+1; i++) {
    for (int j = 0; j < numdiv+1; j++) {
      surface[i][j] = patch.interp (i * step, j * step);
    }
  }

  glBegin(GL_QUADS);
  for (int i = 0; i < numdiv; i++) {
    for (int j = 0; j < numdiv; j++) {
      glLocalGeo (surface[i][j]);
      glLocalGeo (surface[i+1][j]);
      glLocalGeo (surface[i+1][j+1]);
      glLocalGeo (surface[i][j+1]);
    }
  }
  glEnd();

  for (int i = 0; i < numdiv; i++) {
    delete [] surface[i];
  }
  delete [] surface;
}

void renderBezierAdaptive (const BezPatch& patch, float tol) {

  class bp {
    public:
      Vec3 u, x;
      bp (float u, float v, const BezPatch& patch) {
        this->u = Vec3 (u, v, 0);
        this->x = patch.interp (u, v).pos;
      };
      bp (const Vec3& u, const BezPatch& patch)
          : bp (u.x, u.y, patch) {};
  };

  struct tri {
    bp a, b, c;
    tri (const bp& a, const bp& b, const bp& c)
      : a(a), b(b), c(c) {};
  };
  deque<tri> triangles;

  bp p1 (0, 0, patch);
  bp p2 (1, 0, patch);
  bp p3 (1, 1, patch);
  bp p4 (0, 1, patch);
  triangles.emplace_front (p1, p2, p3);
  triangles.emplace_front (p1, p3, p4);

  glBegin(GL_TRIANGLES);
  while (!triangles.empty ()) {
    tri t = triangles.front ();
    triangles.pop_front ();

    bp p1 ((t.a.u + t.b.u) / 2, patch);
    bp p2 ((t.b.u + t.c.u) / 2, patch);
    bp p3 ((t.c.u + t.a.u) / 2, patch);

    float e1 = length (p1.x - (t.a.x + t.b.x) / 2);
    float e2 = length (p2.x - (t.b.x + t.c.x) / 2);
    float e3 = length (p3.x - (t.c.x + t.a.x) / 2);

    if (e1 > tol) {
      if (e2 > tol) {
        if (e3 > tol) {
          triangles.emplace_front (t.a, p1, p3);
          triangles.emplace_front (t.b, p2, p1);
          triangles.emplace_front (t.c, p3, p2);
          triangles.emplace_front (p1, p2, p3);
        } else {
          triangles.emplace_front (t.a, p1, p2);
          triangles.emplace_front (t.b, p2, p1);
          triangles.emplace_front (t.c, t.a, p2);
        }
      } else {
        triangles.emplace_front (t.b, t.c, p1);
        if (e3 > tol) {
          triangles.emplace_front (t.a, p1, p3);
          triangles.emplace_front (t.c, p3, p1);
        } else {
          triangles.emplace_front (t.a, p1, t.c);
        }
      }
    } else {
      if (e2 > tol) {
        triangles.emplace_front (t.b, p2, t.a);
        if (e3 > tol) {
          triangles.emplace_front (t.a, p2, p3);
          triangles.emplace_front (t.c, p3, p2);
        } else {
          triangles.emplace_front (t.a, p2, t.c);
        }
      } else {
        if (e3 > tol) {
          triangles.emplace_front (t.a, t.b, p3);
          triangles.emplace_front (t.b, t.c, p3);
        } else {
          glLocalGeo (patch.interp (t.a.u.x, t.a.u.y));
          glLocalGeo (patch.interp (t.b.u.x, t.b.u.y));
          glLocalGeo (patch.interp (t.c.u.x, t.c.u.y));
        }
      }
    }
  }
  glEnd();
}

//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void display( GLFWwindow* window )
{
  glClearColor( 0.0f, 0.0f, 0.0f, 0.0f ); //clear background screen to black
  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                // clear the color buffer (sets everything to black)
  glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
  glLoadIdentity();                            // make sure transformation is "zero'd"
  
  //----------------------- code to draw objects --------------------------
  glPushMatrix();
  glOrtho(
    viewboxPos.x - viewboxSize * zoom, viewboxPos.x + viewboxSize * zoom,
    viewboxPos.y - viewboxSize * zoom, viewboxPos.y + viewboxSize * zoom,
    viewboxPos.z + viewboxSize * zoom, viewboxPos.z - viewboxSize * zoom);
  glTranslatef (translation[0], translation[1], translation[2]);
  glRotatef (rotation[0], 0, 1, 0);
  glRotatef (rotation[1], 1, 0, 0);

  glEnable (GL_LIGHTING);
  glEnable (GL_LIGHT0);

  for (std::vector<BezPatch>::iterator it = patches.begin ();
       it != patches.end ();
       it++) {
    if (adaptive_subdivision)
      renderBezierAdaptive (*it, subdivisionParam);
    else
      renderBezierUniform (*it, subdivisionParam);
  }

  glPopMatrix();
  
  glfwSwapBuffers(window);

  // note: check out glPolygonMode and glShadeModel 
  // for wireframe and shading commands
  
}

//****************************************************
// function that is called when window is resized
//***************************************************
void size_callback(GLFWwindow* window, int width, int height)
{
    // Get the pixel coordinate of the window
    // it returns the size, in pixels, of the framebuffer of the specified window
    glfwGetFramebufferSize(window, &Width_global, &Height_global);
    
    glViewport(0, 0, Width_global, Height_global);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, Width_global * zoom, 0, Height_global * zoom, 1, -1);
    
    display(window);
}


//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {

  /* Command line args */

  if (argc < 3) {
    cerr << "Error: incorrect number of parameters (minimum 2).\n";
    return -1;
  }

  BezFile bezfile (argv[1]);
  while (bezfile.numPatches () > 0) {
    BezPatch patch;
    bezfile.nextPatch (&patch);
    patches.push_back (patch);
  }
  subdivisionParam = stof (argv[2]);

  if (argc > 3) {
    if (strcmp (argv[3], "-a") == 0)
      adaptive_subdivision = true;
  }

  /* Set clipping planes. */
  Vec3 leftBottomFar, rightTopNear;
  for (std::vector<BezPatch>::iterator it = patches.begin ();
       it != patches.end ();
       it++) {
    Vec3 minBB = it->minBB ();
    if (minBB.x < leftBottomFar.x) leftBottomFar.x = minBB.x;
    if (minBB.y < leftBottomFar.y) leftBottomFar.y = minBB.y;
    if (minBB.z < leftBottomFar.z) leftBottomFar.z = minBB.z;

    Vec3 maxBB = it->maxBB ();
    if (maxBB.x > rightTopNear.x) rightTopNear.x = maxBB.x;
    if (maxBB.y > rightTopNear.y) rightTopNear.y = maxBB.y;
    if (maxBB.z > rightTopNear.z) rightTopNear.z = maxBB.z;
  }

  Vec3 diag = rightTopNear - leftBottomFar;
  viewboxPos = leftBottomFar + diag / 2;
  viewboxSize = diag.x;
  if (diag.y > viewboxSize) viewboxSize = diag.y;
  if (diag.z > viewboxSize) viewboxSize = diag.z;

  //This initializes glfw
  initializeRendering();
  
  GLFWwindow* window = glfwCreateWindow( Width_global, Height_global, "CS184", NULL, NULL );
  if ( !window )
  {
      cerr << "Error on window creating" << endl;
      glfwTerminate();
      return -1;
  }

  const GLFWvidmode * mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
  if ( !mode )
  {
      cerr << "Error on getting monitor" << endl;
      glfwTerminate();
      return -1;
  }
  
  glfwMakeContextCurrent( window );
  
  // Get the pixel coordinate of the window
  // it returns the size, in pixels, of the framebuffer of the specified window
  glfwGetFramebufferSize(window, &Width_global, &Height_global);
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glEnable(GL_DEPTH_TEST);	// enable z-buffering
  glDepthFunc(GL_LESS);

  glfwSetWindowTitle(window, "CS184");
  glfwSetWindowSizeCallback(window, size_callback);
  glfwSetKeyCallback(window, key_callback);

  while ( !glfwWindowShouldClose( window ) ) // infinite loop to draw object again and again
  {   // because once object is draw then window is terminated
      display( window );
      
      if (auto_strech){
          glfwSetWindowSize(window, mode->width, mode->height);
          glfwSetWindowPos(window, 0, 0);
      }
      
      glfwPollEvents();
  }

  return 0;
}