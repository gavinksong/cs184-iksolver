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