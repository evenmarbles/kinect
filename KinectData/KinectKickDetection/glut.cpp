#include "glut.h"
#include "main.h"

static int menu_id;
static int outputmenu_id;
static int guardmenu_id;
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
		guarded_data = TRUE;
	}
	else if (num == 4){
		guarded_data = FALSE;
	}
	else if (num == 5){
		pickled_data = FALSE;
	}
	else if (num == 6){
		pickled_data = TRUE;
	}
	else if (num == 7){
		record_data = TRUE;
	}
	else if (num == 8){
		record_data = FALSE;
	}
	else if (num == 9){
		save_data = TRUE;
	}
	glutPostRedisplay();
	menu_used = TRUE;
}

void createMenu(void){
	legmenu_id = glutCreateMenu(menu);
	glutAddMenuEntry("Left", 1);
	glutAddMenuEntry("Right", 2);

	guardmenu_id = glutCreateMenu(menu);
	glutAddMenuEntry("Enabled", 3);
	glutAddMenuEntry("Disabled", 4);
	
	outputmenu_id = glutCreateMenu(menu);
	glutAddMenuEntry("Text File", 5);
	glutAddMenuEntry("Python Pickle", 6);

	recordmenu_id = glutCreateMenu(menu);
	glutAddMenuEntry("Start", 7);
	glutAddMenuEntry("Stop", 8);
	glutAddMenuEntry("Save", 9); 

	menu_id = glutCreateMenu(menu);
	glutAddSubMenu("Select Leg", legmenu_id);
	glutAddSubMenu("Guards", guardmenu_id);
	glutAddSubMenu("Formatting", outputmenu_id);
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