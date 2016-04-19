#pragma once

#include "ofMain.h"

#define TUNE_PID

#ifdef TUNE_PID
#include "ofxGui.h"
#endif

#include "PinArrayController.h"

//#include "ofxOpenCv.h"

class ofApp : public ofBaseApp{

	public:

		////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////// PIN CONTROL //////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////

		~ofApp();

		void setup();
		void update();
		void draw();

		/*
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		*/

		////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////// GRAPHICS /////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////


		void drawInfo();

		int appSelect = 2;
		void armApp();
		void geoApp();
		void simApp();
		void plateApp();
		void honeyApp();
		void waveApp();


		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		bool overCircle(int x, int y, int diameter);
		bool overSquare(int x, int y, int width);
		void animateTo(int current, int final, int elasticFactor);
		int rangeMap(int r1min, int r1max, int r2min, int r2max, int r1val);


		//Pins
		int pins = 12;
		int padding = 420;
		void setPinsNCoords();

		//RESET at setup
		int pinWidth = (ofGetWindowWidth() - (padding * 2)) / pins;
		int pinHeight;
		int allPinsWidth = pins*pinWidth;
		int pinElasticity = 50;
		int pinBaseline = pinHeight;
		////////////////


		////////////////////////////////////////////////////////////////
		//CONTROLLER
		bool pinsTouched[12];
		int pinsHeights[12];
		int motorMaxPosition = POSITION_MAX;//90000;

		//SIMULATION
		int pinSimForces[12];
		int pinSimHeights[12];
		////////////////////////////////////////////////////////////////


		int pinCoordsX[12];
		int pinCoordsY[12];

		void updatePinHeights();
		void updatePinCoords();
		void updatePinCoordsMouse();

		int mouseHoverRadius = 40;
		ofFbo projectionFbo;
		ofPixels projectionPixels;
		int projectionX = -100;
		int projectionY = 0;

		ofPolyline lineHorz;
		ofPolyline lineVert;
		ofPolyline pinsMin;
		ofPolyline pinsMax;

		bool mouseControl = false;
		bool showTheInfo = true;
		bool showTheForce = false;
		bool showAssist = false;


		//////////////////// APPS ////////////////////

		double armInitCoordsY[12];
		ofImage armImage;
		int armCoordsY1[12];
		int armCoordsY1Disp[12];
		int armCoordsY2[12];
		int skinElasticity = 10;
		int armThresh = 80;
		int armForceMapRadius[3] = { allPinsWidth / 6 , allPinsWidth / 3 , 700 };
		int armForceMapX[3] = { (ofGetWindowWidth() / 2) + 5, ofGetWindowWidth() / 2, ofGetWindowWidth() / 2 };
		int armForceMapY[3] = { pinBaseline + 30, pinBaseline, pinBaseline };
		int armForceMapStrength[3] = { 255, 50, 150 };
		int rocksX[100];
		int rocksY[100];

		ofImage rig;
		ofPolyline geoGrassline;
		int geoInitCoordsY[12];
		int geoInitCoordsYrock[12];
		int geoCoordsY[12];
		int geoCoordsYrock[12];
		int geoThresh = 70;

		ofImage plateLeft;
		ofImage plateRight;
		int plateLeftOffset = 200;
		int plateRightOffset = 200;
		int faultVal = 0;
		double volcanoScale = 0.0;
		double volcanoScaleX = 0.0;
		double tunnelScale = 0.0;
		ofSoundPlayer sound_volcano;
		ofSoundPlayer sound_lava;

		bool volcano = false;
		bool volcanoWaiting = false;
		int volcanoTopCoord = 0;
		ofImage magma2;
		ofImage magma1;
		int magma2Opacity = 0;
		bool magma2Complete = false;
		ofImage water;
		int plateInitCoordsY[12];

		int testInitCoordsY[12];
		int testCoordsY[12];
		int testForceMapStrength[12];


		int honeyInitCoordsY[12];
		int honeyCoordsY[12];
		int honeyViscosity = 4;



		//////////////////// APP FORCE MAPS ////////////////////
		ofFbo geoForce;
		ofFbo armForce;
		ofFbo testForce;
		ofFbo plateForce;
		ofFbo honeyForce;

		void updateGeoForce();
		void updateArmForce();
		void updateTestForce();
		void updatePlateForce();
		void updateHoneyForce();

		ofPixels forcePixels;
		void updatePinForces();


		int pinPressuresBuffer[NUM_DRIVES];

	private:
#ifdef TUNE_PID
		ofxIntSlider   loopPeriodSlider;
		ofxFloatSlider pGainSlider;
		ofxFloatSlider dGainSlider;
		ofxFloatSlider iGainSlider;
		ofxIntSlider   debounceActivationTimeSlider;
		ofxIntSlider   debounceDeactivationTimeSlider;
#ifdef USE_PID_LINEARIZATION
		ofxFloatSlider PIDsetpointMultiplierSlider;
#endif
		ofxPanel gui;
#endif
		PinArrayController* inForce;
};
