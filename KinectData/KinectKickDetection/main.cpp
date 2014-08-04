// based on https://github.com/kyzyx/Tutorials/tree/master/KinectSDK
//
// requires freeglut and glew 



#include <Windows.h>
#include <Ole2.h>

#include <gl/GL.h>
#include <gl/GLU.h>
#include <gl/glut.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <string>
#include <array>

#include "main.h"
#include "glut.h"
#include "VectorClasses.h"



bool testTracked = FALSE;
char* outputText = "empty";
std::ofstream output_file;

struct data_stream_node {
	float zRotAngle;
	float xRotAngle;
	float facingAngle;
	data_stream_node * next;
};

data_stream_node * data_stream;
data_stream_node * current_node;

 
// Left leg is 1. Right leg is 2. Default leg is left.
int active_leg = 1;

// Program status globals
bool record_data = FALSE;
bool save_data = FALSE;
bool guarded_data = FALSE;
bool pickled_data = TRUE;
bool menu_used = FALSE;
bool guard_trap = FALSE;


// OpenGL Variables
GLuint textureId;              // ID of the texture to contain Kinect RGB Data
GLubyte data[width*height*4];  // BGRA array containing the texture data

// Kinect variables
HANDLE rgbStream;              // The identifier of the Kinect's RGB Camera
HANDLE depthStream;			   // The identifier of the Kinect's Depth Sensor
INuiSensor* sensor;            // The kinect sensor

// Stores the coordinates of each joint
Vector4 skeletonPosition[NUI_SKELETON_POSITION_COUNT];





bool initKinect() {
	// Get a working kinect sensor
	int numSensors;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
	if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

	// Initialize sensor
	sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_SKELETON);
	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
		0,        // Image stream flags, e.g. near mode
		2,        // Number of frames to buffer
		NULL,     // Event handle
		&depthStream);
	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
		0,      // Image stream flags, e.g. near mode
		2,      // Number of frames to buffer
		NULL,   // Event handle
		&rgbStream);
	sensor->NuiSkeletonTrackingEnable(NULL, 0); // NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT for only upper body
	return sensor;
}

void getKinectData(GLubyte* dest) {
    NUI_IMAGE_FRAME imageFrame;
    NUI_LOCKED_RECT LockedRect;
    if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) < 0) return;
    INuiFrameTexture* texture = imageFrame.pFrameTexture;
    texture->LockRect(0, &LockedRect, NULL, 0);
    if (LockedRect.Pitch != 0)
    {
        const BYTE* curr = (const BYTE*) LockedRect.pBits;
        const BYTE* dataEnd = curr + (width*height)*4;

        while (curr < dataEnd) {
            *dest++ = *curr++;
        }
    }
    texture->UnlockRect(0);
    sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
}

void getSkeletalData() {
	NUI_SKELETON_FRAME skeletonFrame = { 0 };
	if (sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0) {
		//sensor->NuiTransformSmooth(&skeletonFrame, NULL);
		// Loop over all sensed skeletons
		for (int z = 0; z < NUI_SKELETON_COUNT; ++z) {
			const NUI_SKELETON_DATA& skeleton = skeletonFrame.SkeletonData[z];
			// Check the state of the skeleton
			if (skeleton.eTrackingState == NUI_SKELETON_TRACKED || skeleton.eTrackingState == NUI_SKELETON_POSITION_ONLY) {
				testTracked = TRUE;
				// Copy the joint positions into our array
				for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
					skeletonPosition[i] = skeleton.SkeletonPositions[i];
					if (skeleton.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_NOT_TRACKED) {
						skeletonPosition[i].w = 0;
					}
					else if (skeleton.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_INFERRED) {
						skeletonPosition[i].w = 0; // change this value to 1 if you want inferred joints to be drawn onto the screen
					}
					else if (skeleton.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_TRACKED){
						skeletonPosition[i].w = 1;
					}
				}
				return; // Only take the data for one skeleton
			}
			else testTracked = FALSE;
		}
	}
}

GLfloat* SkeletonToScreen(Vector4 skeletonPoint)
{
	LONG x, y;
	USHORT depth;

	// Calculate the skeleton's position on the screen
	// NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
	NuiTransformSkeletonToDepthImage(skeletonPoint, &x, &y, &depth);

	float screenPointX = static_cast<float>(x * width) / 320;
	float screenPointY = static_cast<float>(y * height) / 240;

	GLfloat pointIn2D[2] = { screenPointX, screenPointY };

	return pointIn2D;
}

float threePointAngle(Vector4 pointA, Vector4 pointMid, Vector4 pointB)
{
	// Calculating the relative angle of two joints given by 3 connected points
	Vector3D& vectorJoint1toJoint2 = Vector3D(pointMid.x - pointA.x, pointMid.y - pointA.y, pointMid.z - pointA.z);
	Vector3D& vectorJoint2toJoint3 = Vector3D(pointMid.x - pointB.x, pointMid.y - pointB.y, pointMid.z - pointB.z);
	vectorJoint1toJoint2.Normalize();
	vectorJoint2toJoint3.Normalize();

	Vector3D crossProduct = vectorJoint1toJoint2 % vectorJoint2toJoint3;
	float crossProductLength = crossProduct.z;
	float dotProduct = vectorJoint1toJoint2 * vectorJoint2toJoint3;
	float segmentAngle = atan2(crossProductLength, dotProduct);

	float degreesJoints = segmentAngle * (180 / M_PI);

	return fabsf(degreesJoints);
}

float twoVectorAngle(Vector3D vectorA, Vector3D vectorB, Vector3D planeNormal)
{
	// Calculating the relative angle of two joints given their vectors


	vectorA.Normalize();
	vectorB.Normalize();	

	// The minimum angle between the vectors
	float dotProduct = vectorA * vectorB;
	float segmentAngle = acos(dotProduct);

	// Determining the sign
	Vector3D crossProduct = vectorA % vectorB;
	float angleSign = planeNormal * crossProduct;
	if (angleSign < 0){
		segmentAngle = -segmentAngle;
	}

	// uncomment for degrees
	// segmentAngle = segmentAngle *(180 / M_PI);

	return segmentAngle;

}

// fill data structure
void appendData(float zRotAngle, float xRotAngle, float facingAngle){
	data_stream_node * new_node;
	new_node = new data_stream_node;
	new_node->zRotAngle = zRotAngle;
	new_node->xRotAngle = xRotAngle;
	new_node->facingAngle = facingAngle;
	new_node->next = NULL;
	current_node->next = new_node;
	current_node = new_node;
}

void writePickle(){

	int data_length = 0;
	current_node = data_stream;
	// find length of linked list
	while (current_node->next != NULL){
		data_length++;
		current_node = current_node->next;
	}
	// create arrays to hold data
	float * zRotArray;
	zRotArray = new float[data_length];
	float * xRotArray;
	xRotArray = new float[data_length];
	float * facingAngle;
	facingAngle = new float[data_length];

	// fill arrays, first node holds no data
	current_node = data_stream->next;
	data_stream_node * temp_node;
	for (int i = 0; i < data_length - 1; i++){
		zRotArray[i] = current_node->zRotAngle;
		xRotArray[i] = current_node->xRotAngle;
		facingAngle[i] = current_node->facingAngle;
		temp_node = current_node;
		current_node = temp_node->next;
		delete temp_node;
	}
	current_node = data_stream;
	
	// construct file name from time
	time_t rawtime;
	struct tm timeinfo;
	char buffer[80];

	time(&rawtime);
	localtime_s(&timeinfo, &rawtime);
	strftime(buffer, 80, "Kinect %Y_%m_%d %H-%M-%S", &timeinfo);
	
	// create txt file
	if (pickled_data == FALSE){
		std::string file_name = std::string(buffer) + ".txt";
		output_file.open(file_name);
		output_file << std::flush;
		output_file << "frames," << std::endl;
		output_file << data_length - 1 << "," << std::endl;
		output_file << "zRotAngle," << std::endl;
		for (int i = 0; i < data_length - 1; i++){
			output_file << zRotArray[i] << ",";
		}
		output_file << std::endl;
		output_file << "xRotAngle," << std::endl;
		for (int i = 0; i < data_length - 1; i++){
			output_file << xRotArray[i] << ",";
		}
		output_file << std::endl;		
		output_file << "facingAngle," << std::endl;
		for (int i = 0; i < data_length - 1; i++){
			if (i < data_length - 2){
				output_file << facingAngle[i] << ",";
			}
			else {
				output_file << facingAngle[i] << std::endl;
			}
		}
		output_file.close();
	}
	// create python pickle file
	else { // pickled_data == TRUE
		std::string file_name = std::string(buffer) + ".p";
		output_file.open(file_name, std::ofstream::binary);
		output_file << std::flush;
		output_file << "(dp0" << "\n";
		output_file << "S'frames'" << "\n";
		output_file << "p1" << "\n";
		output_file << "I" << data_length - 1 << "\n";
		output_file << "sS'zRotAngle'" << "\n";
		output_file << "p2" << "\n";
		output_file << "(lp3" << "\n";
		output_file << "F" << zRotArray[0] << "\n";
		for (int i = 1; i < data_length - 1; i++){
			output_file << "aF" << zRotArray[i] << "\n";
		}
		output_file << "asS'xRotAngle'" << "\n";
		output_file << "p4" << "\n";
		output_file << "(lp5" << "\n";
		output_file << "F" << xRotArray[0] << "\n";
		for (int i = 1; i < data_length - 1; i++){
			output_file << "aF" << xRotArray[i] << "\n";
		}
		output_file << "asS'facingAngle'" << "\n";
		output_file << "p6" << "\n";
		output_file << "(lp7" << "\n";
		output_file << "F" << facingAngle[0] << "\n";
		for (int i = 1; i < data_length - 1; i++){
			output_file << "aF" << facingAngle[i] << "\n";
		}
		output_file << "as.";
		output_file.close();
	}
}

void drawKinectData() {
    glBindTexture(GL_TEXTURE_2D, textureId);
    getKinectData(data);
	getSkeletalData();
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)data);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBegin(GL_QUADS);
	glColor3f(1.f, 1.f, 1.f);
        glTexCoord2f(0.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glTexCoord2f(1.0f, 0.0f);
        glVertex3f(width, 0, 0);
        glTexCoord2f(1.0f, 1.0f);
        glVertex3f(width, height, 0.0f);
        glTexCoord2f(0.0f, 1.0f);
        glVertex3f(0, height, 0.0f);
    glEnd();

	// Torso & Head
	const Vector4& hip = skeletonPosition[NUI_SKELETON_POSITION_HIP_CENTER];
	const Vector4& spn = skeletonPosition[NUI_SKELETON_POSITION_SPINE];
	const Vector4& shc = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_CENTER];
	const Vector4& head = skeletonPosition[NUI_SKELETON_POSITION_HEAD];
	// Left Arm etc
	const Vector4& lh = skeletonPosition[NUI_SKELETON_POSITION_HAND_LEFT];
	const Vector4& le = skeletonPosition[NUI_SKELETON_POSITION_ELBOW_LEFT];
	const Vector4& lw = skeletonPosition[NUI_SKELETON_POSITION_WRIST_LEFT];
	const Vector4& ls = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_LEFT];
	// Right Arm etc
	const Vector4& rh = skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT];
	const Vector4& re = skeletonPosition[NUI_SKELETON_POSITION_ELBOW_RIGHT];
	const Vector4& rw = skeletonPosition[NUI_SKELETON_POSITION_WRIST_RIGHT];
	const Vector4& rs = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_RIGHT];
	// Left Leg etc
	const Vector4& lhip = skeletonPosition[NUI_SKELETON_POSITION_HIP_LEFT];
	const Vector4& lk = skeletonPosition[NUI_SKELETON_POSITION_KNEE_LEFT];
	const Vector4& la = skeletonPosition[NUI_SKELETON_POSITION_ANKLE_LEFT];
	const Vector4& lf = skeletonPosition[NUI_SKELETON_POSITION_FOOT_LEFT];
	// Right Leg etc
	const Vector4& rhip = skeletonPosition[NUI_SKELETON_POSITION_HIP_RIGHT];
	const Vector4& rk = skeletonPosition[NUI_SKELETON_POSITION_KNEE_RIGHT];
	const Vector4& ra = skeletonPosition[NUI_SKELETON_POSITION_ANKLE_RIGHT];
	const Vector4& rf = skeletonPosition[NUI_SKELETON_POSITION_FOOT_RIGHT];
	
	glLineWidth(20.0);
	glBegin(GL_LINES);
	glColor3f(1.f, 0.f, 0.f);

	if (testTracked){

		// Torso & Head
		if (hip.w == 1 && spn.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(hip)[0], SkeletonToScreen(hip)[1]);
			glVertex2f(SkeletonToScreen(spn)[0], SkeletonToScreen(spn)[1]);
		}
		if (spn.w == 1 && shc.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(spn)[0], SkeletonToScreen(spn)[1]);
			glVertex2f(SkeletonToScreen(shc)[0], SkeletonToScreen(shc)[1]);
		}
		if (shc.w == 1 && head.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(shc)[0], SkeletonToScreen(shc)[1]);
			glVertex2f(SkeletonToScreen(head)[0], SkeletonToScreen(head)[1]);
		}
		// Left Arm etc
		if (lh.w == 1 && lw.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(lh)[0], SkeletonToScreen(lh)[1]);
			glVertex2f(SkeletonToScreen(lw)[0], SkeletonToScreen(lw)[1]);
		}
		if (lw.w == 1 && le.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(lw)[0], SkeletonToScreen(lw)[1]);
			glVertex2f(SkeletonToScreen(le)[0], SkeletonToScreen(le)[1]);
		}
		if (le.w == 1 && ls.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(le)[0], SkeletonToScreen(le)[1]);
			glVertex2f(SkeletonToScreen(ls)[0], SkeletonToScreen(ls)[1]);
		}
		if (ls.w == 1 && shc.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(ls)[0], SkeletonToScreen(ls)[1]);
			glVertex2f(SkeletonToScreen(shc)[0], SkeletonToScreen(shc)[1]);
		}
		// Right Arm etc
		if (rh.w == 1 && rw.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(rh)[0], SkeletonToScreen(rh)[1]);
			glVertex2f(SkeletonToScreen(rw)[0], SkeletonToScreen(rw)[1]);
		}
		if (rw.w == 1 && re.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(rw)[0], SkeletonToScreen(rw)[1]);
			glVertex2f(SkeletonToScreen(re)[0], SkeletonToScreen(re)[1]);
		}
		if (re.w == 1 && rs.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(re)[0], SkeletonToScreen(re)[1]);
			glVertex2f(SkeletonToScreen(rs)[0], SkeletonToScreen(rs)[1]);
		}
		if (rs.w == 1 && shc.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(rs)[0], SkeletonToScreen(rs)[1]);
			glVertex2f(SkeletonToScreen(shc)[0], SkeletonToScreen(shc)[1]);
		}
		// Left Leg etc
		if (lf.w == 1 && la.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(lf)[0], SkeletonToScreen(lf)[1]);
			glVertex2f(SkeletonToScreen(la)[0], SkeletonToScreen(la)[1]);
		}
		if (la.w == 1 && lk.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(la)[0], SkeletonToScreen(la)[1]);
			glVertex2f(SkeletonToScreen(lk)[0], SkeletonToScreen(lk)[1]);
		}
		if (lk.w == 1 && lhip.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(lk)[0], SkeletonToScreen(lk)[1]);
			glVertex2f(SkeletonToScreen(lhip)[0], SkeletonToScreen(lhip)[1]);
		}
		if (lhip.w == 1 && hip.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(lhip)[0], SkeletonToScreen(lhip)[1]);
			glVertex2f(SkeletonToScreen(hip)[0], SkeletonToScreen(hip)[1]);
		}
		// Right Leg etc
		if (rf.w == 1 && ra.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(rf)[0], SkeletonToScreen(rf)[1]);
			glVertex2f(SkeletonToScreen(ra)[0], SkeletonToScreen(ra)[1]);
		}
		if (ra.w == 1 && rk.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(ra)[0], SkeletonToScreen(ra)[1]);
			glVertex2f(SkeletonToScreen(rk)[0], SkeletonToScreen(rk)[1]);
		}
		if (rk.w == 1 && rhip.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(rk)[0], SkeletonToScreen(rk)[1]);
			glVertex2f(SkeletonToScreen(rhip)[0], SkeletonToScreen(rhip)[1]);
		}
		if (rhip.w == 1 && hip.w == 1){ // checking to see if both points are tracked
			glVertex2f(SkeletonToScreen(rhip)[0], SkeletonToScreen(rhip)[1]);
			glVertex2f(SkeletonToScreen(hip)[0], SkeletonToScreen(hip)[1]);
		}
	}
	glEnd();

	// Leg angles and their output to screen.
	float fVal1 = 0;
	float fVal2 = 0;
	float facingAngle = 0;
	if (testTracked && rhip.w == 1 && lhip.w == 1 && rk.w == 1 && lk.w == 1){ // checking to see if all the needed joints are tracked
		Vector3D* UpperLegRotZ = NULL;
		Vector3D* UpperLegRotX = NULL;

		int screen_position[2];
		if (active_leg == 1){ // Left leg is selected. Default value.
			UpperLegRotZ = new Vector3D(lk.x - lhip.x, lk.y - lhip.y, 0);
			UpperLegRotX = new Vector3D(0, lk.y - lhip.y, lk.z - lhip.z);
			screen_position[0] = SkeletonToScreen(lhip)[0];
			screen_position[1] = SkeletonToScreen(lhip)[1];
		}
		else if (active_leg == 2){ // Left leg is selected.
			UpperLegRotZ = new Vector3D(rk.x - rhip.x, rk.y - rhip.y, 0);
			UpperLegRotX = new Vector3D(0, rk.y - rhip.y, rk.z - rhip.z);
			screen_position[0] = SkeletonToScreen(rhip)[0]+20; // 20 pixels moved to center it better above joint
			screen_position[1] = SkeletonToScreen(rhip)[1];
		}
		// Creating vector to measure leg angle against and calculating the angles for rotation around z-achsis and x-achsis.		
		Vector3D& jointAttachedPendulum = Vector3D(0, -1, 0);
		Vector3D& planeNormal = Vector3D(0, 0, 1);
		fVal1 = twoVectorAngle(*UpperLegRotZ, jointAttachedPendulum, planeNormal);
		planeNormal = Vector3D(-1, 0, 0);
		fVal2 = twoVectorAngle(*UpperLegRotX, jointAttachedPendulum, planeNormal); 
		// Constructing the calculation for a camera facing vector
		Vector3D& rightHipPointToLeft = Vector3D(lhip.x - rhip.x, 0, lhip.z - rhip.z); 
		planeNormal = Vector3D(0, -1, 0);
		Vector3D& straightFacing = Vector3D(-1, 0, 0); // must align with rightHipPointToLeftVector for skeleton to be facing straight forward
		facingAngle = twoVectorAngle(rightHipPointToLeft, straightFacing, planeNormal);

		delete UpperLegRotZ;
		UpperLegRotZ = NULL;
		delete UpperLegRotX;
		UpperLegRotX = NULL;

		// Write angle of the rotation around z to screen by converting float to char array and printing to screen above the joints.
		glColor3f(0.f, 1.f, 0.f);
		glRasterPos2f(screen_position[0] - 30, screen_position[1] - 20);
		char cVal[32];
		sprintf_s(cVal, "%f", fVal1);
		outputText = cVal;
		for (int i = 0; i < 5; i++) {
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, outputText[i]);
		}
		// New screen position for angle of the rotation around x.
		glRasterPos2f(screen_position[0] - 30, screen_position[1] - 40);
		sprintf_s(cVal, "%f", fVal2);
		outputText = cVal;
		for (int i = 0; i < 5; i++) {
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, outputText[i]);
		}
		// screen position for facing angle
		glRasterPos2f(545, 140);
		sprintf_s(cVal, "%f", facingAngle);
		outputText = cVal;
		for (int i = 0; i < 5; i++) {
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, outputText[i]);
		}		

		glLineWidth(5.0);
		glBegin(GL_LINES);
		// blue arrow for facing
		glColor3f(0.f, 0.f, 1.f);
		glLineWidth(5.0);
		// arrow shaft
		glVertex2f(570 + sin(facingAngle) * 50, 70 - cos(facingAngle) * 50);
		glVertex2f(570 - sin(facingAngle) * 50 , 70 + cos(facingAngle) * 50); 
		// arrow tip part 1
		glVertex2f(570 - sin(facingAngle+0.4) * 30, 70 + cos(facingAngle+0.4) * 30);
		glVertex2f(570 - sin(facingAngle) * 50, 70 + cos(facingAngle) * 50);
		// arrow tip part 2
		glVertex2f(570 - sin(facingAngle - 0.4) * 30, 70 + cos(facingAngle - 0.4) * 30);
		glVertex2f(570 - sin(facingAngle) * 50, 70 + cos(facingAngle) * 50);
		glEnd();

	}
	// Nag message about menu appears on screen if menu has not been used yet.
	if (menu_used == FALSE){
		outputText = "Left click on screen to use menu!";
		glColor3f(255, 0, 0);
		glRasterPos2f(10, 20);
		int len;
		len = (int)strlen(outputText);
		for (int i = 0; i < len; i++) {
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, outputText[i]);
		}
	}
	// Guards:
	// catch legs that are not fully tracked and bad angles
	if (lk.w == 0 || lhip.w == 0){
		guard_trap = TRUE;
	}
	if (fVal1 > 45.29){
		guard_trap = TRUE;
	}
	if (guarded_data == FALSE){
		guard_trap = FALSE;
	}
	// Program actions
	if (record_data == TRUE && save_data == FALSE){
		appendData(fVal1, fVal2, facingAngle);
	}
	else if (record_data == TRUE && save_data == TRUE && guard_trap == FALSE){
		record_data = FALSE;
		save_data = FALSE;
		appendData(fVal1, fVal2, facingAngle);
		writePickle();
	}
	else if (record_data == FALSE && save_data == TRUE && data_stream->next != NULL && guard_trap == FALSE){
		save_data == FALSE;
		writePickle();
	}

}







int main(int argc, char* argv[]) {
    if (!init(argc, argv)) return 1;
    if (!initKinect()) return 1;
	
	// Setting up the data structure
	data_stream = new data_stream_node;
	data_stream->next = NULL;
	data_stream->zRotAngle = 0;
	data_stream->xRotAngle = 0;
	data_stream->facingAngle = 0;
	current_node = data_stream;
	
    // Initialize textures
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*) data);
    glBindTexture(GL_TEXTURE_2D, 0);

    // OpenGL setup
    glClearColor(0,0,0,0);
    glClearDepth(1.0f);
    glEnable(GL_TEXTURE_2D);

    // Camera setup
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, height, 0, 1, -1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Main loop
    execute();
    return 0;
}
