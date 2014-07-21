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

#include "main.h"
#include "glut.h"
#include "VectorClasses.h"



bool testTracked = FALSE;
char* outputText = "empty";
std::ofstream output_file;

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

float twoVectorAngle(Vector3D vectorA, Vector3D vectorB)
{
	// Calculating the relative angle of two joints given their vectors
	vectorA.Normalize();
	vectorB.Normalize();

	Vector3D crossProduct = vectorA % vectorB;
	float crossProductLength = crossProduct.x;
	float dotProduct = vectorA * vectorB;
	float segmentAngle = atan2(crossProductLength, dotProduct);

	float degreesJoints = segmentAngle * (180 / M_PI);

	//return fabsf(degreesJoints);
	return degreesJoints;
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
		glVertex2f(SkeletonToScreen(hip)[0], SkeletonToScreen(hip)[1]);
		glVertex2f(SkeletonToScreen(spn)[0], SkeletonToScreen(spn)[1]);

		glVertex2f(SkeletonToScreen(spn)[0], SkeletonToScreen(spn)[1]);
		glVertex2f(SkeletonToScreen(shc)[0], SkeletonToScreen(shc)[1]);

		glVertex2f(SkeletonToScreen(shc)[0], SkeletonToScreen(shc)[1]);
		glVertex2f(SkeletonToScreen(head)[0], SkeletonToScreen(head)[1]);

		// Left Arm etc
		glVertex2f(SkeletonToScreen(lh)[0], SkeletonToScreen(lh)[1]);
		glVertex2f(SkeletonToScreen(lw)[0], SkeletonToScreen(lw)[1]);

		glVertex2f(SkeletonToScreen(lw)[0], SkeletonToScreen(lw)[1]);
		glVertex2f(SkeletonToScreen(le)[0], SkeletonToScreen(le)[1]);

		glVertex2f(SkeletonToScreen(le)[0], SkeletonToScreen(le)[1]);
		glVertex2f(SkeletonToScreen(ls)[0], SkeletonToScreen(ls)[1]);

		glVertex2f(SkeletonToScreen(ls)[0], SkeletonToScreen(ls)[1]);
		glVertex2f(SkeletonToScreen(shc)[0], SkeletonToScreen(shc)[1]);

		// Right Arm etc
		glVertex2f(SkeletonToScreen(rh)[0], SkeletonToScreen(rh)[1]);
		glVertex2f(SkeletonToScreen(rw)[0], SkeletonToScreen(rw)[1]);

		glVertex2f(SkeletonToScreen(rw)[0], SkeletonToScreen(rw)[1]);
		glVertex2f(SkeletonToScreen(re)[0], SkeletonToScreen(re)[1]);

		glVertex2f(SkeletonToScreen(re)[0], SkeletonToScreen(re)[1]);
		glVertex2f(SkeletonToScreen(rs)[0], SkeletonToScreen(rs)[1]);

		glVertex2f(SkeletonToScreen(rs)[0], SkeletonToScreen(rs)[1]);
		glVertex2f(SkeletonToScreen(shc)[0], SkeletonToScreen(shc)[1]);

		// Left Leg etc
		glVertex2f(SkeletonToScreen(lf)[0], SkeletonToScreen(lf)[1]);
		glVertex2f(SkeletonToScreen(la)[0], SkeletonToScreen(la)[1]);

		glVertex2f(SkeletonToScreen(la)[0], SkeletonToScreen(la)[1]);
		glVertex2f(SkeletonToScreen(lk)[0], SkeletonToScreen(lk)[1]);

		glVertex2f(SkeletonToScreen(lk)[0], SkeletonToScreen(lk)[1]);
		glVertex2f(SkeletonToScreen(lhip)[0], SkeletonToScreen(lhip)[1]);

		glVertex2f(SkeletonToScreen(lhip)[0], SkeletonToScreen(lhip)[1]);
		glVertex2f(SkeletonToScreen(hip)[0], SkeletonToScreen(hip)[1]);

		// Right Leg etc
		glVertex2f(SkeletonToScreen(rf)[0], SkeletonToScreen(rf)[1]);
		glVertex2f(SkeletonToScreen(ra)[0], SkeletonToScreen(ra)[1]);

		glVertex2f(SkeletonToScreen(ra)[0], SkeletonToScreen(ra)[1]);
		glVertex2f(SkeletonToScreen(rk)[0], SkeletonToScreen(rk)[1]);

		glVertex2f(SkeletonToScreen(rk)[0], SkeletonToScreen(rk)[1]);
		glVertex2f(SkeletonToScreen(rhip)[0], SkeletonToScreen(rhip)[1]);

		glVertex2f(SkeletonToScreen(rhip)[0], SkeletonToScreen(rhip)[1]);
		glVertex2f(SkeletonToScreen(hip)[0], SkeletonToScreen(hip)[1]);

	}
	glEnd();

	// Screentext
	if (testTracked){
		outputText = "Skeleton tracked!";
		glColor3f(0.f, 1.f, 0.f);	
	
		glRasterPos2f(SkeletonToScreen(lhip)[0]-20, SkeletonToScreen(lhip)[1]-20);
		// z coordinate unchanged (lhip.z) to get angle in x-achsis
		Vector3D& leftUpperLegX = Vector3D(lhip.x - lk.x, lhip.y - lk.y, lhip.z);
		Vector3D& jointAttachedPendulum = Vector3D(0, 1, 0);
		float fVal = twoVectorAngle(leftUpperLegX, jointAttachedPendulum);
		// Write to File
		output_file << fVal << " ";
		// Write to Screen
		char cVal[32];
		sprintf_s(cVal, "%f", fVal);
		outputText = cVal;
		for (int i = 0; i < 4; i++) {
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, outputText[i]);
		}

		glRasterPos2f(SkeletonToScreen(lhip)[0] - 20, SkeletonToScreen(lhip)[1] - 40);
		// x coordinate unchanged (lhip.x) to get angle in z-achsis
		// MATH IS NOT CORRECT IN THE ZY PLANE ANGLE CALCULATION RIGHT NOW! So using z coordinate instead.
		//Vector3D& leftUpperLegZ = Vector3D(lhip.x - lk.x, lhip.y - lk.y, lhip.z);
		fVal = lk.z; //twoVectorAngle(leftUpperLegZ, jointAttachedPendulum);
		// Write to File
		output_file << fVal << std::endl;
		// Write to Screen
		sprintf_s(cVal, "%f", fVal);
		outputText = cVal;
		for (int i = 0; i < 4; i++) {
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, outputText[i]);
		}
	}
	else {
		outputText = "Skeleton not yet tracked!";
		glColor3f(1.f, 0.f, 0.f);
	}
	glRasterPos2f(10, 20);
	int len;
	len = (int)strlen(outputText);
	for (int i = 0; i < len; i++) {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, outputText[i]);
	}

}




int main(int argc, char* argv[]) {
    if (!init(argc, argv)) return 1;
    if (!initKinect()) return 1;

	// Write-to file
	time_t rawtime;
	struct tm timeinfo;
	char buffer[80];

	time(&rawtime);
	localtime_s(&timeinfo, &rawtime);
	strftime(buffer, 80, "Kinect %Y_%m_%d %H-%M-%S", &timeinfo);

	std::string file_name = std::string(buffer) + ".txt";
	output_file.open(file_name); 
	output_file << std::flush;

	//output_file << file_name << std::endl;

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
