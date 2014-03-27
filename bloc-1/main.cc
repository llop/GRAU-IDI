#if defined(__APPLE__)
  #include <OpenGL/OpenGL.h>
  #include <GLUT/GLUT.h>
#else
  #include <GL/gl.h>
  #include <GL/freeglut.h>
#endif
#include <iostream>
using namespace std; 


// dimensions
int wid, hei, siz;


// coords triangle
double coords_triangle[9];
double coords_triangle_tmp[9];

// control coordenades ratoli
int lx, ly;

// background brightness
double br;

bool bg_color_change, new_vertices;

int vertex_num;


void print_help(void) {
  cout << "h   - help" << endl;
  cout << "f   - Activa/desactiva el canvi de color de fons arrossegant el ratolí." << endl;
  cout << "t   - Dibuixa un nou triangle. Fes click on vulguis posar els vèrtexos." << endl;
  cout << "ESC - exit" << endl << endl;
}

void init_constants(void) {
  wid = hei = 600;
  
  coords_triangle[0] = coords_triangle_tmp[0] = -1.0 / 2.0;
  coords_triangle[1] = coords_triangle_tmp[1] = -1.0 / 3.0;
  coords_triangle[2] = coords_triangle_tmp[2] = 0.0;
  coords_triangle[3] = coords_triangle_tmp[3] = 1.0 / 2.0;
  coords_triangle[4] = coords_triangle_tmp[4] = -1.0 / 3.0;
  coords_triangle[5] = coords_triangle_tmp[5] = 0.0;
  coords_triangle[6] = coords_triangle_tmp[6] = 0.0;
  coords_triangle[7] = coords_triangle_tmp[7] = 2.0 / 3.0;
  coords_triangle[8] = coords_triangle_tmp[8] = 0.0;
}

void init_vars(void) {
  br = 0.5;
  bg_color_change = false;
  new_vertices = false;
  vertex_num = 0;
}


void set_cam(void) {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  
  //glOrtho(inspect.lef, inspect.righ, inspect.bottom, inspect.top, inspect.zNear, inspect.zFar);
  
  glMatrixMode(GL_MODELVIEW);
}

void set_position(void) {
  glLoadIdentity();
  
  //glTranslated(0.0, 0.0, -inspect.dist);
  //glRotated(-inspect.zang, 0.0, 0.0, 1.0);
  //glRotated(inspect.xang, 1.0, 0.0, 0.0);
  //glRotated(-inspect.yang, 0.0, 1.0, 0.0);
  //glTranslated(-inspect.vrpx, -inspect.vrpy, -inspect.vrpz);
    
}


void reset_cam(void) {
  set_cam();
  set_position();
  glutPostRedisplay();
}


void paint_triangle(void) {
  glPushMatrix();
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_TRIANGLES);
  
  glColor3d(1.0, 0.0, 0.0);
  glVertex3d(coords_triangle[0], coords_triangle[1], coords_triangle[2]);
  glColor3d(0.0, 1.0, 0.0);
  glVertex3d(coords_triangle[3], coords_triangle[4], coords_triangle[5]);
  glColor3d(0.0, 0.0, 1.0);
  glVertex3d(coords_triangle[6], coords_triangle[7], coords_triangle[8]);
  
  glEnd();
}


void refresh(void) {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  // pintar triangle
  paint_triangle();
  
  glutSwapBuffers();
}


void reshape(int width, int height) {
  if (!height) height = 1;  // no dividir per 0
  if (!width) width = 1;
  wid = width;
  hei = height;
  
  siz = width < height ? width : height;
  glViewport((wid - siz) / 2, (hei - siz) / 2, siz, siz);
  
  set_cam();
  glutPostRedisplay();
}


void initGl(int argc, const char *argv[]) {
  glutInit(&argc, (char**)argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(wid, hei);
  glutCreateWindow("IDI - Bloc 1 OpenGL");
  glClearColor(br, br, br, 1.0);  // black background
  
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
  } else if (key == 'f') {
    if (new_vertices) {
      new_vertices = false;
      vertex_num = 0;
      cout << "Nou triangle cancel·lat" << endl;
    }
    
    bg_color_change = !bg_color_change;
    cout << "Arrossega el ratolí per canviar el color de fons: " << (bg_color_change ? "Activat" : "Desactivat") << endl;
  } else if (key == 't') {
    if (bg_color_change) {
      bg_color_change = false;
      cout << "Canvi del color de fons cancel·lat" << endl;
    }
    new_vertices = !new_vertices;
    cout << "Fes click on vulguis posar els vèrtex d'un nou triangle" << endl;
  }
}

void set_new_vertices(void) {
  coords_triangle[0] = coords_triangle_tmp[0];
  coords_triangle[1] = coords_triangle_tmp[1];
  coords_triangle[2] = coords_triangle_tmp[2];
  coords_triangle[3] = coords_triangle_tmp[3];
  coords_triangle[4] = coords_triangle_tmp[4];
  coords_triangle[5] = coords_triangle_tmp[5];
  coords_triangle[6] = coords_triangle_tmp[6];
  coords_triangle[7] = coords_triangle_tmp[7];
  coords_triangle[8] = coords_triangle_tmp[8];
  
  new_vertices = false;
  vertex_num = 0;
  
  glutPostRedisplay();
}

void mouseHandler(int button, int state, int x, int y) {
  if (/*button == GLUT_LEFT_BUTTON && */state == GLUT_DOWN) {
    // pillar coordenades inicials
    
    if (new_vertices) {
      x -= (wid - siz) / 2.0;
      y = hei - y;
      y -= (hei - siz) / 2.0;
      
      // cout << "Vèrtex " << (3 - lc) << ": " << x << ' ' << y << endl;
      double nx = -1.0 + (x * 2.0 / siz);
      double ny = -1.0 + (y * 2.0 / siz);
      
      //if (nx >= -1.0 && nx <= 1.0 && ny >= -1.0 && ny <= 1.0) {
	// count new vertex
	coords_triangle_tmp[vertex_num * 3] = nx;
	coords_triangle_tmp[vertex_num * 3 + 1] = ny;
	++vertex_num;
	if (vertex_num == 3) {
	  set_new_vertices();
	}
      //} else {
	//cout << "Coordenades fora de límits" << endl;
      //}
    }
    
    lx = x;
    ly = y;
  }
}

void motionHandler(int x, int y) {
  if (bg_color_change) {
    // triangle color
    br += (x - lx) / 600.0;
    if (br < -1.0) {
      br = -1.0;
    } else if (br > 1.0) {
      br = 1.0;
    }
    glClearColor(br, br, br, 1.0);
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
// tar -zcvf bloc4.tar.gz Makefile main.cc
int main(int argc, const char *argv[]) {
  init_constants();
  init_vars();
  initGl(argc, argv);
  setGLCallbacks();
  glutMainLoop();
  return 0;
}