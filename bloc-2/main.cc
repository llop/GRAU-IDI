#if defined(__APPLE__)
  #include <OpenGL/OpenGL.h>
  #include <GLUT/GLUT.h>
#else
  #include <GL/gl.h>
  #include <GL/freeglut.h>
#endif
#include <iostream>
#include <cmath>
#include "model.h"
using namespace std;


typedef vector<Vertex> VertVec;
typedef vector<Face> FaceVec;
typedef vector<int> IntVec;


struct Cam {
  double left, right, top, bottom;
  double zNear, zFar;
  double dist;

  double xAng, yAng, zAng;
  double xVrp, yVrp, zVrp;
  double xUp, yUp, zUp;
  
  Cam() : left(-1.0), right(1.0), top(1.0), bottom(-1.0), zNear(-1.0), zFar(1.0), dist(1.0),
      xAng(0.0), yAng(0.0), zAng(0.0), xVrp(0.0), yVrp(0.0), zVrp(0.0), xUp(0.0), yUp(1.0), zUp(0.0)  {}
  Cam(double left, double right, double top, double bottom, double zNear, double zFar, double dist,
      double xAng, double yAng, double zAng, double xVrp, double yVrp, double zVrp, double xUp, double yUp, double zUp) : 
      left(left), right(right), top(top), bottom(bottom), zNear(zNear), zFar(zFar), dist(dist),
      xAng(xAng), yAng(yAng), zAng(zAng), xVrp(xVrp), yVrp(yVrp), zVrp(zVrp), xUp(xUp), yUp(yUp), zUp(zUp) {}
      
};


//---------------------------------------------------------
//
// Variables
//
//---------------------------------------------------------

// dimensions
int wid, hei;

// coords terra
double coords_floor[] = { -0.75, -0.4, -0.75, -0.75, -0.4, 0.75, 0.75, -0.4, 0.75, 0.75, -0.4, -0.75 };

// flag per decidir que fa el ratoli
bool action;

// control coordenades ratoli
int lx, ly;

// dades camara
Cam cam;

// pel model del legoman
Model model;
double model_box[6];
double model_scale;
double xLego, yLego, zLego;
double xMax, xMin, zMax, zMin;


//---------------------------------------------------------
//
// Angles helper methods
//
//---------------------------------------------------------

const double PI = acos(-1.0);
const double RADIAN_DEGREES = 180.0 / PI;


inline double r2d(const double radians) {
  return radians * RADIAN_DEGREES;
}
inline double d2r(const double degrees) {
  return degrees / RADIAN_DEGREES;
}

inline double correct_angle(double degrees) {
  while (degrees < 0.0) degrees += 360.0;
  while (degrees > 360.0) degrees -= 360.0;
  return degrees;
}


//---------------------------------------------------------
//
// Model helper methods
//
//---------------------------------------------------------

// el vertex te un max o un min?
inline void process_vertex(const Vertex *v, double *box) {
  const double x = *v++;
  if (box[0] < x) box[0] = x;       // nou max x
  else if (box[3] > x) box[3] = x;  // nou min x
  const double y = *v++;
  if (box[1] < y) box[1] = y;
  else if (box[4] > y) box[4] = y;
  const double z = *v;
  if (box[2] < z) box[2] = z;
  else if (box[5] > z) box[5] = z;
}

inline void bounding_box(const Model &m, double *box) {
  const VertVec &vs = m.vertices();
  const FaceVec &fs = m.faces();
  const int fsiz = fs.size();
  if (fsiz) {
    int ind = fs[0].v[0];            // init box to 1st vertex
    box[0] = box[3] = vs[ind++];     // max & min x
    box[1] = box[4] = vs[ind++];     // max & min y
    box[2] = box[5] = vs[ind];       // max & min z
    for (int i = 0; i < fsiz; ++i) {
      const Face &f = fs[i];
      const IntVec &fv = f.v;
      process_vertex(&vs[fv[0]], box);
      process_vertex(&vs[fv[1]], box);
      process_vertex(&vs[fv[2]], box);
    }
  }
}


//---------------------------------------------------------
//
// Print functionality
//
//---------------------------------------------------------

void print_help(void) {
  cout << "h   - help" << endl;
  cout << "c   - Moviment amb el ratolí: Alternar entre rotar l'escena, i moure el legoman." << endl;
  cout << "ESC - exit" << endl << endl;
}


//---------------------------------------------------------
//
// Initialize stuff
//
//---------------------------------------------------------

void init_constants(void) {
  wid = hei = 600;
}

void init_vars(void) {
  // calcular el radi de l'esfera contenidora
  const double d1 = sqrt(0.75 * 0.75 + 0.75 * 0.75);
  const double d = sqrt(d1 * d1 + 0.4 * 0.4);  // 1.0;
  cam = Cam(-d, d, d, -d, -d, d, d, 45.0, -45.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  
  action = false;
}

void init_model() {
  // model load
  model.load("legoman.obj");
  // legoman is 0.5 tall -> calculate scale: need to find bounding box
  bounding_box(model, model_box);
  model_scale = 0.5 / (model_box[1] - model_box[4]);
  // lego positioning
  xLego = 0.75 - model_box[0] * model_scale;
  yLego = -0.4 - model_box[4] * model_scale;
  zLego = 0.75 - model_box[2] * model_scale;
  // limits
  xMax = xLego;
  xMin = -0.75 - model_box[3] * model_scale;
  zMax = zLego;
  zMin = -0.75 - model_box[5] * model_scale;
}


//---------------------------------------------------------
//
// Set camera parameters
//
//---------------------------------------------------------

void set_cam(void) {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glOrtho(cam.left, cam.right, cam.bottom, cam.top, cam.zNear, cam.zFar);

  glMatrixMode(GL_MODELVIEW);
}

void set_position(void) {
  glLoadIdentity();

  glRotated(cam.xAng, 1.0, 0.0, 0.0);
  glRotated(cam.yAng, 0.0, 1.0, 0.0);
  //glRotated(cam.zAng, 0.0, 0.0, 1.0);
  //glTranslated(-cam.xVrp, -cam.yVrp, -cam.zVrp);
}


void reset_cam(void) {
  set_cam();
  set_position();
  glutPostRedisplay();
}


//---------------------------------------------------------
//
// Painting methods
//
//---------------------------------------------------------

inline void paint_floor(void) {
  // definicio color
  glColor3d(0.0, 0.0, 1.0);

  // pintar poligon
  glPushMatrix();
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_QUADS);
  glVertex3dv(&coords_floor[0]);
  glVertex3dv(&coords_floor[3]);
  glVertex3dv(&coords_floor[6]);
  glVertex3dv(&coords_floor[9]);
  glEnd();
  glPopMatrix();
}

inline void paint_snowman_head(void) {
  // definicio color
  glColor3d(1.0, 1.0, 0.0);

  // pintar esfera
  glPushMatrix();
  glTranslated(0.0, 0.6, 0.0);
  glutSolidSphere(0.2, 32, 32);
  glPopMatrix();
}

inline void paint_snowman_nose(void) {
  // definicio color
  glColor3d(1.0, 0.0, 0.0);

  // pintar nas
  glPushMatrix();
  glTranslated(0.1, 0.6, 0.0);
  glRotated(90.0, 0.0, 1.0, 0.0);
  glutSolidCone(0.1, 0.2, 24, 24);
  glPopMatrix();
}

inline void paint_snowman_body(void) {
  // definicio color
  glColor3d(1.0, 1.0, 1.0);

  // pintar esfera
  glPushMatrix();
  glutSolidSphere(0.4, 48, 48);
  glPopMatrix();
}

inline void paint_snowman(void) {
  paint_snowman_head();
  paint_snowman_nose();
  paint_snowman_body();
}

inline void paint_legoman(void) {
  glPushMatrix();
  
  // TGs
  glTranslatef(xLego, yLego, zLego);
  glScaled(model_scale, model_scale, model_scale);
  
  // pinta triangles
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_TRIANGLES);
  const vector<Vertex> &vs = model.vertices();
  const vector<Face> &fs = model.faces();
  const int fsiz = fs.size();
  for (int i = 0; i < fsiz; ++i) {
    const Face &f = fs[i];
    glColor4fv(Materials[f.mat].diffuse);
    glVertex3dv(&vs[f.v[0]]);
    glVertex3dv(&vs[f.v[1]]);
    glVertex3dv(&vs[f.v[2]]);
  }
  glEnd();
  glPopMatrix();
}


//---------------------------------------------------------
//
// Callbacks
//
//---------------------------------------------------------

void refresh(void) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // pintar escena
  paint_floor();
  paint_snowman();
  paint_legoman();

  glutSwapBuffers();
}


void reshape(int width, int height) {
  if (!height) height = 1;  // no dividir per 0
  if (!width) width = 1;
  wid = width;
  hei = height;
  glViewport(0, 0, wid, hei);
  
  // camara axonometrica: calcular top, bottom, left, i right
  const double aspect = double(wid) / hei;
  if (aspect < 1.0) {
    cam.left = -cam.dist;
    cam.right = cam.dist;
    cam.bottom = -cam.dist / aspect;
    cam.top = cam.dist / aspect;
  } else {
    cam.left = -cam.dist * aspect;
    cam.right = cam.dist * aspect;
    cam.bottom = -cam.dist;
    cam.top = cam.dist;
  }
  
  set_cam();
  glutPostRedisplay();
}


void initGl(int argc, const char *argv[]) {
  glutInit(&argc, (char**)argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(wid, hei);
  glutCreateWindow("IDI - Bloc 2 OpenGL");
  glClearColor(0.5, 0.5, 1.0, 1.0);  // light blue background

  // enable stuff
  glEnable(GL_NORMALIZE);
  glEnable(GL_DEPTH_TEST);
  
  // prepare cam
  reset_cam();
}

void keyHandler(unsigned char key, int x, int y) {
  if (key == 27) {
    exit(0);
  } else if (key == 'h') {
    print_help();
  } else if (key == 'c') {
    action = !action;
    cout << (action ? "Ara pots moure el legoman" : "Ara pots rotar l'escena") << endl;
  }
}

void mouseHandler(int button, int state, int x, int y) {
  if (/*button == GLUT_LEFT_BUTTON && */state == GLUT_DOWN) {
    // pillar coordenades inicials
    lx = x;
    ly = y;
  }
}

void motionHandler(int x, int y) {
  if (!action) {
    // rotar
    const double yRot = (x - lx) / 10.0;  // amb 10 la rotacio es prou suau
    const double xRot = (y - ly) / 10.0;
    cam.xAng = correct_angle(cam.xAng + xRot);
    cam.yAng = correct_angle(cam.yAng + yRot);
    set_position();
  } else {
    const double xMov = (x - lx) / 200.0;  // 200 makes the lego move more or less in sync with thw mouse
    const double zMov = (y - ly) / 200.0;
    xLego += xMov;
    zLego += zMov;
    if (xLego < xMin) xLego = xMin;
    else if (xLego > xMax) xLego = xMax;
    if (zLego < zMin) zLego = zMin;
    else if (zLego > zMax) zLego = zMax;
  }
  
  lx = x;
  ly = y;
  glutPostRedisplay();
}

void setGLCallbacks(void) {
  glutDisplayFunc(refresh);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyHandler);
  glutMouseFunc(mouseHandler);
  glutMotionFunc(motionHandler);
}

// per comprimir amb un tar.gz:
// tar -zcvf bloc2.tar.gz Makefile main.cc
int main(int argc, const char *argv[]) {
  
  // inicialitzacions
  init_constants();
  init_vars();
  init_model();
  initGl(argc, argv);
  
  // callbacks
  setGLCallbacks();
  
  // engegar glut
  cout << (action ? "Ara pots moure el legoman" : "Ara pots rotar l'escena") << endl;
  glutMainLoop();
  return 0;
} 
