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
  double dist, zoom;

  double xAng, yAng, zAng;
  double xVrp, yVrp, zVrp;
  double xUp, yUp, zUp;
 
  double xObs, yObs, zObs;
  double fovy, fovyIni, aspect;
 
  Cam() : left(-1.0), right(1.0), top(1.0), bottom(-1.0), zNear(-1.0), zFar(1.0), dist(1.0), zoom(1.0),
      xAng(0.0), yAng(0.0), zAng(0.0), xVrp(0.0), yVrp(0.0), zVrp(0.0), xUp(0.0), yUp(1.0), zUp(0.0),
      xObs(0.0), yObs(0.0), zObs(0.0), fovy(asin(1.0)), fovyIni(asin(1.0)), aspect(1.0) {}
  Cam(double left, double right, double top, double bottom, double zNear, double zFar, double dist, double zoom,
      double xAng, double yAng, double zAng, double xVrp, double yVrp, double zVrp, double xUp, double yUp, double zUp,
      double xObs, double yObs, double zObs, double fovy, double fovyIni, double aspect) :
      left(left), right(right), top(top), bottom(bottom), zNear(zNear), zFar(zFar), dist(dist), zoom(zoom),
      xAng(xAng), yAng(yAng), zAng(zAng), xVrp(xVrp), yVrp(yVrp), zVrp(zVrp), xUp(xUp), yUp(yUp), zUp(zUp),
      xObs(xObs), yObs(yObs), zObs(zObs), fovy(fovy), fovyIni(fovyIni), aspect(aspect) {}
     
};


struct ModelData {
  double xOff, yOff, zOff;
  double scale, rad, front;
 
  double xPos, yPos, zPos;
  double yRot;
  
  double velo;
  
  ModelData() : xOff(0.0), yOff(0.0), zOff(0.0), scale(1.0), rad(0.0), front(0.0), xPos(0.0), yPos(0.0), zPos(0.0), yRot(0.0), velo(0.4) {}
  ModelData(double xOff, double yOff, double zOff, double scale, double rad, double front, double xPos, double yPos, double zPos, double yRot, double velo) :
    xOff(xOff), yOff(yOff), zOff(zOff), scale(scale), rad(rad), front(front), xPos(xPos), yPos(yPos), zPos(zPos), yRot(yRot), velo(velo) {}
};


//---------------------------------------------------------
//
// Variables
//
//---------------------------------------------------------

// dimensions
int wid, hei;
double rad;
double cen[3];

// coords terra
double coords_floor[] = { -5.0, 0.0, -5.0, -5.0, 0.0, 5.0, 5.0, 0.0, 5.0, 5.0, 0.0, -5.0 };

// coords snowmen
double coords_snow[] = { 2.5, 0.0, 2.5, -2.5, 0.0, 2.5, -2.5, 0.0, -2.5 };

// coords walls
double coords_wall[] = { 2.5, 0.75, -1.5, -4.9, 0.75, 0.0 };
double size_wall[] = { 4.0, 1.5, 0.2, 0.2, 1.5, 10.0 };

// flags
bool visible_walls;
bool inspect;
bool axo_cam;
bool tafaner;

// control coordenades ratoli
int lx, ly;

// dades camara
Cam cam, walk;

// pel model del patricio
Model model;
double model_box[6];
ModelData model_datas[2];

// limits
double maxVelo, minVelo;
double maxZoom, minZoom;
double maxXAng, minXAng;
double angRot;


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
// Vectors helper methods
//
//---------------------------------------------------------

// producte cartesia
inline void cross(const double ax, const double ay, const double az, 
		  const double bx, const double by, const double bz, 
		  double &x, double &y, double &z) {
  x = ay * bz - az * by;
  y = az * bx - ax * bz;
  z = ax * by - ay * bx;
}

// vector unitari
inline void normalize(double &x, double &y, double &z) {
  const double l = sqrt(x * x + y * y + z * z);
  if (l > 0.0) {
    x /= l;
    y /= l;
    z /= l;
  }
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

void correct_model_pos(ModelData &model_data) {
  if (model_data.xPos + model_data.rad > 5.0) model_data.xPos = 5.0 - model_data.rad;
  else if (model_data.xPos - model_data.rad < -5.0) model_data.xPos = -5.0 + model_data.rad;
  if (model_data.zPos + model_data.rad > 5.0) model_data.zPos = 5.0 - model_data.rad;
  else if (model_data.zPos - model_data.rad < -5.0) model_data.zPos = -5.0 + model_data.rad;
}

//---------------------------------------------------------
//
// Print functionality
//
//---------------------------------------------------------

void print_help(void) {
  cout << "h   - help" << endl;
  cout << "ESC - exit" << endl;
  cout << "r   - tornar a la vista inicial" << endl;
  cout << "v   - fer visibles/invisibles les parets" << endl;
  cout << "c   - canviar entre mode 'inspecció' i  'walk'" << endl;
  cout << "En mode inspecció:" << endl;
  cout << "------------------" << endl;
  cout << "p   - canviar entre càmara 'axonomètrica' i 'perspectiva'" << endl;
  cout << "Arrastra el ratolí per girar l'escena" << endl;
  cout << "Arrastra el ratolí amunt o avall mentre apretas 'shift' per fer zoom in o out" << endl << endl;
  cout << "En mode walk:" << endl;
  cout << "-------------" << endl;
  cout << "t   - activar/desactivar el mode 'tafaner'" << endl << endl;
  cout << "En mode tafaner:" << endl;
  cout << "----------------" << endl;
  cout << "Fes servir las flechas de direcció per moure la càmara en qualsevol direcció" << endl << endl;
  cout << "Per controlar el moviment del Patricio:" << endl;
  cout << "---------------------------------------" << endl;
  cout << "w   - avançar" << endl;
  cout << "s   - retrocedir" << endl;
  cout << "a   - girar a l'esqerra" << endl;
  cout << "d   - girar a la dreta" << endl << endl;
  cout << "Per controlar la velocitat del Patricio:" << endl;
  cout << "----------------------------------------" << endl;
  cout << "z   - més ràpid" << endl;
  cout << "x   - més lent" << endl << endl;
}


//---------------------------------------------------------
//
// Initialize stuff
//
//---------------------------------------------------------

void init_constants(void) {
  // dimensions i zoom
  wid = hei = 600;
  maxZoom = 0.05;
  minZoom = 5.0;
  maxXAng = 89.99;
  minXAng = -89.99;
  angRot = 7.5;
  maxVelo = 1.5;
  minVelo = 0.1;
  
  // calcul automatic del radi de l'esfera contenidora
  const double maxx = 5.0;
  const double maxy = 1.5;
  const double maxz = 5.0;
  const double minx = -5.0;
  const double miny = 0.0;
  const double minz = -5.0;
  const double xval = maxx - minx;
  const double yval = maxy - miny;
  const double zval = maxz - minz;
  const double d1 = sqrt(xval * xval + yval * yval);
  rad = sqrt(zval * zval + d1 * d1) / 2.0;
  cen[0] = (maxx + minx) / 2.0;
  cen[1] = (maxy + miny) / 2.0;
  cen[2] = (maxz + minz) / 2.0;
}

void apply_aspect(void) {
  if (cam.aspect < 1.0) {
    // camara axonometrica
    cam.left = -rad;
    cam.right = rad;
    cam.bottom = -rad / cam.aspect;
    cam.top = rad / cam.aspect;
    // camara perspectiva: calcular fovy
    cam.fovy = atan(tan(cam.fovyIni) / cam.aspect);
  } else {
    cam.left = -rad * cam.aspect;
    cam.right = rad * cam.aspect;
    cam.bottom = -rad;
    cam.top = rad;
  }
}

void init_vars(void) {
 
  // camara mode inspeccio
  cam = Cam(-rad, rad, rad, -rad, rad, rad * 3.0, rad * 2.0, 1.0,
        0.0, 0.0, 0.0, cen[0], cen[1], cen[2], 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, asin(0.5), asin(0.5), (double)wid / hei);
  
  // pel walk partir de la camara de inspeccio
  // fovy de 60 graus per la cam del 'walk'
  // zNear també a pinyon  
  walk = cam;
  walk.fovyIni = d2r(60.0);
  walk.zNear = 0.001;
  walk.xAng = walk.yAng = 0.0;
  
  // observador inicial
  walk.xObs = walk.zObs = 0.0;
  walk.yObs = 0.5;
  
  apply_aspect();
  
  visible_walls = true;
  inspect = true;
  axo_cam = true;
  tafaner = false;
  
}

void init_model() {
 
  // carregar model + capsa contenidora
  model.load("Patricio.obj");
  bounding_box(model, model_box);
 
  // quina es la dimensio mes gran?
  const double xs = model_box[0] - model_box[3];
  const double ys = model_box[1] - model_box[4];
  const double zs = model_box[2] - model_box[5];
  double maxd = xs > ys ? xs : ys;
  maxd = zs > maxd ? zs : maxd;
 
  // patricio #1
  double scale = 1.0 / maxd;
  double rad = sqrt(xs * xs + zs * zs) * scale / 2.0;
  double xOff = -(model_box[0] + model_box[3]) * scale / 2.0;
  double yOff = -model_box[4] * scale;
  double zOff = -(model_box[2] + model_box[5]) * scale / 2.0;
  double front = model_box[2] * scale + zOff;
  model_datas[0] = ModelData(xOff, yOff, zOff, scale, rad, front, 0.0, 0.0, 0.0, 0.0, 0.4);
  
  // patricio #2
  scale = 1.5 / maxd;
  rad = sqrt(xs * xs + zs * zs) * scale / 2.0;
  xOff = -(model_box[0] + model_box[3]) * scale / 2.0;
  yOff = -model_box[4] * scale;
  zOff = -(model_box[2] + model_box[5]) * scale / 2.0;
  front = model_box[2] * scale + zOff;
  model_datas[1] = ModelData(xOff, yOff, zOff, scale, rad, front, 2.5, 0.0, -2.5, 90.0, 0.4);
 
}


//---------------------------------------------------------
//
// Set camera parameters
//
//---------------------------------------------------------

void set_cam(void) {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
   
  if (inspect) {
    // mode inspeccio: tenim camara axonometrica i perspectiva
    if (axo_cam) {
      glOrtho(cam.left * cam.zoom, cam.right * cam.zoom, cam.bottom * cam.zoom, cam.top * cam.zoom, cam.zNear, cam.zFar);
      
    } else {
      const double angle = r2d(2.0 * cam.fovy);
      gluPerspective(angle * cam.zoom, cam.aspect, cam.zNear, cam.zFar);
    }
   
  } else {
    // mode 'walk'
    const double angle = r2d(2.0 * walk.fovy);
    gluPerspective(angle / walk.zoom, walk.aspect, walk.zNear, walk.zFar);
  }
 
  glMatrixMode(GL_MODELVIEW);
}

void set_position(void) {
  if (inspect) {
    // mode inspeccio
    double m[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, m);
    glLoadIdentity();
    glTranslated(0.0, 0.0, -cam.dist);
    glRotated(cam.xAng, 1.0, 0.0, 0.0);
    glRotated(cam.yAng, 0.0, 1.0, 0.0);
    glTranslated(0.0, 0.0, cam.dist);
    glMultMatrixd(m);
    cam.xAng = cam.yAng = 0.0;
  } else {
    // mode 'walk'
    // calcular el vector look (és unitari)
    const double xLook = cos(d2r(walk.xAng)) * sin(d2r(correct_angle(model_datas[0].yRot + walk.yAng)));
    const double yLook = sin(d2r(walk.xAng));
    const double zLook = cos(d2r(walk.xAng)) * cos(d2r(correct_angle(model_datas[0].yRot + walk.yAng)));
    
    // l'OBS ve donat per la posicio del patricio#1
    walk.xObs = model_datas[0].xPos + model_datas[0].front * xLook;
    walk.zObs = model_datas[0].zPos + model_datas[0].front * zLook;
    
    // calcular la distancia des de l'observador fins el borde de la esfera contenidora, seguint el vector look
    const double a = xLook * xLook + yLook * yLook + zLook * zLook;
    const double b = 2.0 * (xLook * walk.xObs + yLook * walk.yObs + zLook * walk.zObs);
    const double c = walk.xObs * walk.xObs + walk.yObs * walk.yObs + walk.zObs * walk.zObs - rad * rad;
    const double d = b * b - 4.0 * a * c;
    
    // pillar només la meitat, que alla hi va el VRP
    // ara podriem setejar zFar amb exactitud, però no ho fem perque no cal ;)
    walk.dist = (sqrt(d) - b) / (4.0 * a);
    
    // punt de referencia a la distancia calculada
    walk.xVrp = walk.xObs + walk.dist * xLook;
    walk.yVrp = walk.yObs + walk.dist * yLook;
    walk.zVrp = walk.zObs + walk.dist * zLook;
    
    // calcular vector up
    double rx, ry, rz;
    // producte cartesia del vector de visio i (0, 1, 0) -> right vector
    cross(xLook, yLook, zLook, 0.0, 1.0, 0.0, rx, ry, rz);
    // normalize(rx, ry, rz);
    // producte cartesia del right i el look = up
    cross(rx, ry, rz, xLook, yLook, zLook, walk.xUp, walk.yUp, walk.zUp);
    
    // normalize(walk.xUp, walk.yUp, walk.zUp);
    gluLookAt(walk.xObs, walk.yObs, walk.zObs, walk.xVrp, walk.yVrp, walk.zVrp, walk.xUp, walk.yUp, walk.zUp);
  }
}


void reset_cam(void) {
  set_cam();
  
  // transformacions inicials
  glLoadIdentity();
  glTranslated(0.0, 0.0, -cam.dist);
  glRotated(45.0, 1.0, 0.0, 0.0);
  glRotated(-45.0, 0.0, 1.0, 0.0);
  glTranslated(-cam.xVrp, -cam.yVrp, -cam.zVrp);
  
  glutPostRedisplay();
}


//---------------------------------------------------------
//
// Painting methods
//
//---------------------------------------------------------

inline void paint_floor(void) {
  // definicio color
  glColor3d(0.5, 0.0, 0.0);

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
  glTranslated(0.0, 1.0, 0.0);
  glutSolidSphere(0.2, 32, 32);
  glPopMatrix();
}

inline void paint_snowman_nose(void) {
  // definicio color
  glColor3d(1.0, 0.0, 0.0);

  // pintar nas
  glPushMatrix();
  glTranslated(0.1, 1.0, 0.0);
  glRotated(90.0, 0.0, 1.0, 0.0);
  glutSolidCone(0.1, 0.2, 24, 24);
  glPopMatrix();
}

inline void paint_snowman_body(void) {
  // definicio color
  glColor3d(1.0, 1.0, 1.0);

  // pintar esfera
  glPushMatrix();
  glTranslated(0.0, 0.4, 0.0);
  glutSolidSphere(0.4, 48, 48);
  glPopMatrix();
}

inline void paint_snowman(const double xPos, const double yPos, const double zPos) {
  glPushMatrix();
  glTranslated(xPos, yPos, zPos);
  paint_snowman_head();
  paint_snowman_nose();
  paint_snowman_body();
  glPopMatrix();
}

inline void paint_snowman(const double *pos) {
  paint_snowman(*pos, *(pos + 1), *(pos + 2));
}

inline void paint_wall(const double xPos, const double yPos, const double zPos, const double xSiz, const double ySiz, const double zSiz) {
  // definicio color
  glColor3d(0.0, 0.5, 0.0);
 
  glPushMatrix();
  glTranslated(xPos, yPos, zPos);
  glScaled(xSiz, ySiz, zSiz);
  glutSolidCube(1.0);
  glPopMatrix();
}

inline void paint_wall(const double *pos, const double *siz) {
  paint_wall(*pos, *(pos + 1), *(pos + 2), *siz, *(siz + 1), *(siz + 2));
}

inline void paint_patricio(const ModelData &model_data) {
  glPushMatrix();
 
  // Posicio i rotacio
  glTranslated(model_data.xPos, model_data.yPos, model_data.zPos);
  glRotated(model_data.yRot, 0.0, 1.0, 0.0);
 
  // Colocar el patricio al centre i escalat
  glTranslated(model_data.xOff, model_data.yOff, model_data.zOff);
  glScaled(model_data.scale, model_data.scale, model_data.scale);
 
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

  paint_floor();
 
  if (visible_walls) {
    paint_wall(&coords_wall[0], &size_wall[0]);
    paint_wall(&coords_wall[3], &size_wall[3]);
  }
 
  paint_snowman(&coords_snow[0]);
  paint_snowman(&coords_snow[3]);
  paint_snowman(&coords_snow[6]);
 
  paint_patricio(model_datas[0]);
  paint_patricio(model_datas[1]);
 
  glutSwapBuffers();
}

void reshape(int width, int height) {
  if (!height) height = 1;  // no dividir per 0
  if (!width) width = 1;
  wid = width;
  hei = height;
  glViewport(0, 0, wid, hei);
 
  cam.aspect = walk.aspect = double(wid) / hei;
  apply_aspect();
 
  set_cam();
  glutPostRedisplay();
}


void initGl(int argc, const char *argv[]) {
  glutInit(&argc, (char**)argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(wid, hei);
  glutCreateWindow("IDI - Bloc 3 OpenGL");
  glClearColor(0.0, 0.0, 0.0, 1.0);  // black background

  // enable stuff
  glEnable(GL_NORMALIZE);
  glEnable(GL_DEPTH_TEST);
}

void keyHandler(unsigned char key, int x, int y) {
  bool do_refresh = false;
  if (key == 27) {
    exit(0);
   
  } else if (key == 'h') {
    print_help();
   
  } else if ((do_refresh = key == 'v')) {
    visible_walls = !visible_walls;
    cout << "Parets " << (visible_walls ? "visibles" : "invisibles") << endl;
   
  } else if ((do_refresh = inspect && key == 'p')) {
    axo_cam = !axo_cam;
    set_cam();
    cout << "Camara " << (axo_cam ? "axonometrica" : "perspectiva") << endl;
   
  } else if ((do_refresh = key == 'r')) {
    init_vars();
    reset_cam();
    cout << "Configuracio inicial de l'escena" << endl;
   
  } else if ((do_refresh = key == 'c')) {
    inspect = !inspect;
    set_cam();
    set_position();
    cout << "Mode " << (inspect ? "inspeccio" : "'walk'") << endl;
    
  } else if (!inspect && key == 't') {
    tafaner = !tafaner;
    cout << "Mode tafaner " << (tafaner ? "activat" : "desactivat") << endl;
    
  } else if (key == 'z') {
    model_datas[0].velo += 0.1;
    if (model_datas[0].velo > maxVelo) model_datas[0].velo = maxVelo;
    
  } else if (key == 'x') {
    model_datas[0].velo -= 0.1;
    if (model_datas[0].velo < minVelo) model_datas[0].velo = minVelo;
    
  } else if ((do_refresh = key == 'a')) {
    model_datas[0].yRot = correct_angle(model_datas[0].yRot + angRot);
    set_position();
    
  } else if ((do_refresh = key == 'd')) {
    model_datas[0].yRot = correct_angle(model_datas[0].yRot - angRot);
    set_position();
    
  } else if ((do_refresh = key == 'w')) {
    const double xLook = sin(d2r(model_datas[0].yRot));
    const double zLook = cos(d2r(model_datas[0].yRot));
    model_datas[0].xPos += model_datas[0].velo * xLook;
    model_datas[0].zPos += model_datas[0].velo * zLook;
    correct_model_pos(model_datas[0]);
    set_position();
    
  } else if ((do_refresh = key == 's')) {
    const double xLook = sin(d2r(model_datas[0].yRot));
    const double zLook = cos(d2r(model_datas[0].yRot));
    model_datas[0].xPos -= model_datas[0].velo * xLook;
    model_datas[0].zPos -= model_datas[0].velo * zLook;
    correct_model_pos(model_datas[0]);
    set_position();
  }
  
  if (do_refresh) glutPostRedisplay();
}

void keySpecial(int key, int x, int y) {
  if (!inspect && tafaner) {
    bool do_refresh = false;
    if ((do_refresh = key == GLUT_KEY_LEFT)) {
      walk.yAng = correct_angle(walk.yAng + angRot);
      
    } else if ((do_refresh = key == GLUT_KEY_RIGHT)) {
      walk.yAng = correct_angle(walk.yAng - angRot);
      
    } else if ((do_refresh = key == GLUT_KEY_UP)) {
      walk.xAng = walk.xAng + angRot;
      if (walk.xAng > maxXAng) walk.xAng = maxXAng;
      
    } else if ((do_refresh = key == GLUT_KEY_DOWN)) {
      walk.xAng = walk.xAng - angRot;
      if (walk.xAng < minXAng) walk.xAng = minXAng;
    }
    
    // repintar si algo canvia
    if (do_refresh) {
      set_position();
      glutPostRedisplay();
    }
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
  // mode inspeccio: permet rotar l'escena o fer zoom si s'apreta el shift
  if (inspect) {
   
    if (glutGetModifiers() == GLUT_ACTIVE_SHIFT) {
      // fer zoom
      const double yOff = (y - ly) / 500.0;  // dividir per donar-li la "sensibilitat" adequada
      cam.zoom += yOff;
      // limitar el zoom
      if (cam.zoom > minZoom) cam.zoom = minZoom;
      else if (cam.zoom < maxZoom) cam.zoom = maxZoom;
      set_cam();
     
    } else {
      // rotar
      const double yRot = (x - lx) / 10.0;  // amb 10 la rotacio es prou suau
      const double xRot = (y - ly) / 10.0;
      cam.xAng = correct_angle(xRot);
      cam.yAng = correct_angle(yRot);
      set_position();
     
    }
  }
 
  lx = x;
  ly = y;
  glutPostRedisplay();
}

void setGLCallbacks(void) {
  glutDisplayFunc(refresh);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyHandler);
  glutSpecialFunc(keySpecial);
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
  
  // prepare cam
  reset_cam();
 
  // callbacks
  setGLCallbacks();
 
  // engegar glut
  glutMainLoop();
  return 0;
} 