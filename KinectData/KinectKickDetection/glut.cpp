#include "glut.h"
#include "main.h"

static int menu_id;
static int recordmenu_id;
static int legmenu_id;
static int window;
static int value = 0;

void menu(int num){
	if (num == 0){
		glutDestroyWindow(window);
		exit(0);
	}
	else if (num == 1){
		active_leg = 1;
	}
	else if (num == 2){
		active_leg = 2;
	}
	else if (num == 3){
		record_data = TRUE;
	}
	else if (num == 4){
		record_data = FALSE;
	}
	else if (num == 5){
		save_data = TRUE;
	}
	glutPostRedisplay();
}

void createMenu(void){
	recordmenu_id = glutCreateMenu(menu);
	glutAddMenuEntry("Start", 3);
	glutAddMenuEntry("Stop", 4);
	glutAddMenuEntry("Save", 5); 

	legmenu_id = glutCreateMenu(menu);
	glutAddMenuEntry("Left", 1);
	glutAddMenuEntry("Right", 2);

	menu_id = glutCreateMenu(menu);
	glutAddSubMenu("Select Leg", legmenu_id);
	glutAddSubMenu("Recording", recordmenu_id);
	glutAddMenuEntry("Quit", 0);
	
	glutAttachMenu(GLUT_LEFT_BUTTON);
}

void draw() {
	drawKinectData();
	glutSwapBuffers();
}

void execute() {
	glutMainLoop();
}

bool init(int argc, char* argv[]) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(width, height);
	window = glutCreateWindow("Kinect Skeleton");
	createMenu();
	glutDisplayFunc(draw);
	glutIdleFunc(draw);
	return true;
}