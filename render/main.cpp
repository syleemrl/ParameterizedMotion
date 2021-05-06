#include <string>
#include <iostream>
#include <boost/program_options.hpp>
#include <GL/glut.h>
#include <QApplication>
#include <QGLWidget>
#include "MainWindow.h"
class GLWidget : public QGLWidget{
    void initializeGL(){
        glClearColor(0.0, 1.0, 1.0, 1.0);
    }
    
    void qgluPerspective(GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar){
        const GLdouble ymax = zNear * tan(fovy * M_PI / 360.0);
        const GLdouble ymin = -ymax;
        const GLdouble xmin = ymin * aspect;
        const GLdouble xmax = ymax * aspect;
        glFrustum(xmin, xmax, ymin, ymax, zNear, zFar);
    }
    
    void resizeGL(int width, int height){
        if (height==0) height=1;
        glViewport(0,0,width,height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        qgluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,100.0f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
     }
    
    void paintGL(){
        glMatrixMode(GL_MODELVIEW);         
        glLoadIdentity();
        glClear(GL_COLOR_BUFFER_BIT);  

        glBegin(GL_POLYGON); 
            glVertex2f(-0.5, -0.5); 
            glVertex2f(-0.5, 0.5);
            glVertex2f(0.5, 0.5); 
            glVertex2f(0.5, -0.5); 
        glEnd();
    }
};
int main(int argc,char** argv)
{
	boost::program_options::options_description desc("allowed options");
	desc.add_options()
	("bvh,b",boost::program_options::value<std::string>())
	("ppo,p",boost::program_options::value<std::string>())
    ("parametric", "parametric");

	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	std::string bvh="", ppo="";
    bool parametric = false;

	if(vm.count("bvh")) {
		bvh = vm["bvh"].as<std::string>();
	}
	if(vm.count("ppo")) {
		ppo = vm["ppo"].as<std::string>();
	}
    if(vm.count("parametric")) {
        parametric = true;
    }

	glutInit(&argc,argv);
	QApplication a(argc, argv);
    
    MainWindow* main_window = new MainWindow(bvh, ppo, parametric);
    main_window->resize(2560,1440);
    main_window->show();
    return a.exec();
}
