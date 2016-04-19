#include "ofApp.h"

#include "PinArrayController.h"

//--------------------------------------------------------------
void ofApp::setup(){

	////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////// GRAPHICS //////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////

	ofBackground(0, 0, 0);
	// ofBackground(255,255,255);

	setPinsNCoords();

	//Draw some Center Lines and other image shit like that
	armImage.load("arm2.png");
	rig.load("rig.png");
	plateLeft.load("plateleft.png");
	plateRight.load("plateRight.png");
	magma1.load("magma1.png");
	magma2.load("magma2.png");
	water.load("water.png");

	ofEnableAlphaBlending();
	lineHorz.addVertex(0, ofGetWindowHeight() / 2);
	lineHorz.addVertex(ofGetWindowWidth(), ofGetWindowHeight() / 2);
	lineVert.addVertex(ofGetWindowWidth() / 2, 0);
	lineVert.addVertex(ofGetWindowWidth() / 2, ofGetWindowHeight());

	//Setup Force Maps
	forcePixels.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGB);
	projectionPixels.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGB);
	projectionFbo.allocate(ofGetWindowWidth(), ofGetWindowHeight());

	geoForce.allocate(ofGetWindowWidth(), ofGetWindowHeight());
	armForce.allocate(ofGetWindowWidth(), ofGetWindowHeight());
	testForce.allocate(ofGetWindowWidth(), ofGetWindowHeight());
	plateForce.allocate(ofGetWindowWidth(), ofGetWindowHeight());
	honeyForce.allocate(ofGetWindowWidth(), ofGetWindowHeight());

	geoForce.begin();
	ofClear(255, 255, 255, 0);
	geoForce.end();

	plateForce.begin();
	ofClear(255, 255, 255, 0);
	plateForce.end();

	armForce.begin();
	ofClear(255, 255, 255, 0);
	armForce.end();

	testForce.begin();
	ofClear(255, 255, 255, 0);
	testForce.end();

	projectionFbo.begin();
	ofClear(255, 255, 255, 0);
	projectionFbo.end();

	honeyForce.begin();
	ofClear(255,255,255, 0);
	honeyForce.end();


	sound_volcano.load("volcano.mp3");
	sound_lava.load("lava.mp3");
	sound_lava.setLoop(true);

	for (int i = 0; i < 100; i++) {
		rocksX[i] = (int)ofRandom((pinCoordsX[0] - (pinWidth / 2)), ofGetWindowWidth() - padding);
		rocksY[i] = (int)ofRandom(pinHeight*0.5, pinBaseline);
	}
    
    cout << "STUFF";


	////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////// PIN CONTROL //////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////

	inForce = new PinArrayController(NUM_DRIVES, DEFAULT_LOOP_PERIOD_MICROSECONDS);

	// Set up GUI stuff
#ifdef TUNE_PID
	gui.setup();
	gui.add(loopPeriodSlider.setup("LOOP PERIOD", DEFAULT_LOOP_PERIOD_MICROSECONDS, MIN_LOOP_PERIOD, MAX_LOOP_PERIOD));
	gui.add(pGainSlider.setup("PROPORTIONAL GAIN", DEFAULT_PROPORTIONAL_GAIN, 0, PROPORTAIONAL_TUNING_MAX));
	gui.add(dGainSlider.setup("DERIVATIVE GAIN", DEFAULT_DERIVATIVE_GAIN, DERIVATIVE_TUNING_MIN, DERIVATIVE_TUNING_MAX));
	gui.add(iGainSlider.setup("INTEGRAL GAIN", DEFAULT_INTEGRAL_GAIN, 0, INTEGRAL_TUNING_MAX));
	gui.add(debounceActivationTimeSlider.setup("DEBOUNCE ACTVTN", DEFAULT_PIN_ACTIVATION_TOUCH_DEBOUNCE_WINDOW_US,MIN_DEBOUNCE_TIME_ACTIVATION, MAX_DEBOUNCE_TIME_ACTIVATION));
	gui.add(debounceDeactivationTimeSlider.setup("DEBOUNCE DEACTVTN", DEFAULT_PIN_DEACTIVATION_TOUCH_DEBOUNCE_WINDOW_US, MIN_DEBOUNCE_TIME_DEACTIVATION, MAX_DEBOUNCE_TIME_DEACTIVATION));
#ifdef USE_PID_LINEARIZATION
	gui.add(PIDsetpointMultiplierSlider.setup("SETPOINT MULT", DEFAULT_PID_SETPOINT_MULTIPLIER, MIN_PID_SETPOINT_MULTIPLIER, MAX_PID_SETPOINT_MULTIPLIER));
#endif
	gui.setPosition(200,100);
#endif

	inForce->startPins();
	//(inForce->getPinController())->testMultipleDrives();
	//(inForce->getPinController())->testSendAllPinsToTop();
	inForce->startThread();
}

// destructor
ofApp::~ofApp() {
	delete inForce;
}

//--------------------------------------------------------------
void ofApp::update(){
	////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////// GRAPHICS /////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////
	// is pinsTouched?


	switch (appSelect)
	{
	case 0:
		updateArmForce();
		break;
	case 1:
		updateGeoForce();
		break;
	case 2:
		updateTestForce();
		break;
	case 3:
		updatePlateForce();
		break;
	case 4:
		updateHoneyForce();
		break;
	case 5:

		break;
	default:
		updateTestForce();
		break;
	}

	///////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////// PIN CONTROL /////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////

#ifdef TUNE_PID
	inForce->setLoopPeriod(loopPeriodSlider);
	inForce->setPIDGains(pGainSlider, iGainSlider, dGainSlider);
	inForce->setPinTouchDebounceTimes(debounceActivationTimeSlider, debounceDeactivationTimeSlider);
#ifdef USE_PID_LINEARIZATION
	inForce->setPIDSetPointMultiplier(PIDsetpointMultiplierSlider);
#endif

#endif

	// Get current states of pins
	inForce->getPinActivations(pinsTouched);
	inForce->getSensedPinPressures(pinPressuresBuffer);
	inForce->getCommandedPinPositions(pinsHeights);

	// update model state
	updatePinForces();
	updatePinHeights();
	if (mouseControl) {
		updatePinCoordsMouse();
	}
	else {
		updatePinCoords();
	}

	// Send model state to pin controller
	inForce->setDesiredPinForces(pinSimForces);
	inForce->setDesiredPinHeights(pinSimHeights);


	// NOTE: On startup the pins will "jump" once - this is because the initial desired heights (40) do not correspond to POSITION_MAX, which is what the controller's pin heights are initialized to.
	// The pins end up jumping temporatily to 108758 (pixel height 40) from 120000 (Position Max)
}

//--------------------------------------------------------------
void ofApp::draw(){
	////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////// GRAPHICS /////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////
	projectionFbo.begin();
	ofClear(255, 255, 255, 0);

	switch (appSelect)
	{
	case 0:
		armApp();
		break;
	case 1:
		geoApp();
		break;
	case 2:
		simApp();
		break;
	case 3:
		plateApp();
		break;
	case 4:
	  honeyApp();
	  break;
	case 5:
		waveApp();
	  break;
	default:
		simApp();
	}

	drawInfo();

	projectionFbo.end();
	projectionFbo.readToPixels(projectionPixels);

	projectionFbo.draw(projectionX, projectionY);


	////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////// PIN CONTROL /////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////
#ifdef TUNE_PID
	gui.draw();
#endif
}

////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// GRAPHICS /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

void ofApp::updateArmForce() {

	armForce.begin();

	ofClear(255, 255, 255, 0);

	ofFill();

	ofSetColor(255 - armForceMapStrength[2], 255 - armForceMapStrength[2], 255 - armForceMapStrength[2], 255);
	ofSetPolyMode(OF_POLY_WINDING_NONZERO);

	ofBeginShape();
	ofCurveVertex(pinCoordsX[0], armCoordsY1[0]);
	for (int i = 0; i < pins; i++) {
		ofCurveVertex(pinCoordsX[i], armCoordsY1[i]);
	}
	for (int i = pins - 2; i > -1; i--) {
		ofCurveVertex(pinCoordsX[i], (pinBaseline - armInitCoordsY[i] + pinHeight));
	}
	ofEndShape();

	ofCurveVertex(pinCoordsX[0], armCoordsY1[0]);

	for (int i = 1; i > -1; i--) {
		ofFill();
		//        ofSetColor(255, 0, 0, 255);
		//        ofDrawCircle(armForceMapX[i] ,armForceMapY[i] , armForceMapRadius[i]);
		ofSetColor(255 - armForceMapStrength[i], 255 - armForceMapStrength[i], 255 - armForceMapStrength[i], 255);
		ofDrawCircle(armForceMapX[i], armForceMapY[i], armForceMapRadius[i]);
	}

	armForce.end();

	armForce.readToPixels(forcePixels);

}

void ofApp::updateGeoForce() {

	geoForce.begin();
	ofClear(255, 255, 255, 0);


	geoForce.begin();
	ofClear(255, 255, 255, 0);

	//////ADDME///////

	//Draw Ocean
	ofSetColor(230, 230, 230, 255);
	ofFill();
	ofDrawRectangle((ofGetWindowWidth() / 2) - (allPinsWidth / 2), geoInitCoordsY[0] + 30, allPinsWidth - 3, pinHeight);

	//Draw Earth Layer
	ofSetColor(123, 92, 64, 255);
	ofFill();
	ofBeginShape();
	for (int i = 0; i < pins; i++) {

		if (i == 0) {
			ofCurveVertex((pinCoordsX[i] - (pinWidth / 2)), geoCoordsY[i]);
		}
		else if (i == (pins - 1)) {
			ofCurveVertex((pinCoordsX[i] + (pinWidth / 2)), geoCoordsY[i]);
		}
		else if (i == 1) {
			ofCurveVertex((pinCoordsX[i] + (pinWidth / 2)), geoCoordsY[i] + 140);
		}
		else {
			ofCurveVertex(pinCoordsX[i], geoCoordsY[i]);
		}

	}
	ofCurveVertex(pinCoordsX[11] + (pinWidth / 2), geoCoordsY[11] + 2);
	ofVertex(pinCoordsX[11] + (pinWidth / 2), ofGetWindowHeight());
	ofVertex(pinCoordsX[0] - (pinWidth / 2), ofGetWindowHeight());
	ofVertex(pinCoordsX[0] - (pinWidth / 2), geoCoordsY[0]);
	ofEndShape(FALSE);


	//Draw Grass Layer
	geoGrassline.clear();
	geoGrassline.addVertex((pinCoordsX[0] - (pinWidth / 2)), geoCoordsY[0] + 15);
	for (int i = 0; i < pins; i++) {

		if (i == (pins - 1)) {
			geoGrassline.curveTo((pinCoordsX[i] + (pinWidth / 2)), geoCoordsY[i] + 15);
		}
		else if (i == 1) {
			geoGrassline.curveTo((pinCoordsX[i] + (pinWidth / 2)), geoCoordsY[i] + 155);
		}
		else {
			geoGrassline.curveTo(pinCoordsX[i], geoCoordsY[i] + 15);
		}

	}
	geoGrassline.addVertex(pinCoordsX[11] + (pinWidth / 2), geoCoordsY[11] + 15);
	ofSetColor(50, 50, 50, 255);
	glEnable(GL_POINT_SMOOTH);
	glLineWidth(30);
	ofSetPolyMode(OF_POLY_WINDING_NONZERO);

	//  ofSetLineWidth(30);
	ofNoFill();
	geoGrassline.draw();


	//Draw Sediment Layer
	ofSetColor(200, 200, 200, 255);
	ofFill();
	ofBeginShape();
	for (int i = 0; i < pins; i++) {

		if (i == 0) {
			ofCurveVertex((pinCoordsX[i] - (pinWidth / 2)), geoCoordsYrock[i]);
		}
		else if (i == (pins - 1)) {
			ofCurveVertex((pinCoordsX[i] + (pinWidth / 2)), geoCoordsYrock[i]);
		}
		else {
			ofCurveVertex(pinCoordsX[i], geoCoordsYrock[i]);
		}

	}
	ofCurveVertex(pinCoordsX[11] + (pinWidth / 2), geoCoordsYrock[11] + 2);
	ofVertex(pinCoordsX[11] + (pinWidth / 2), ofGetWindowHeight());
	ofVertex(pinCoordsX[0] - (pinWidth / 2), ofGetWindowHeight());
	ofVertex(pinCoordsX[0] - (pinWidth / 2), geoCoordsYrock[0]);
	ofEndShape(FALSE);


	//Draw Oil
	ofSetColor(230, 230, 230, 255);
	ofFill();
	ofBeginShape();
	ofCurveVertex(pinCoordsX[0] - pinWidth / 2, geoCoordsYrock[0] + 100);
	ofCurveVertex(pinCoordsX[0] - pinWidth / 2, geoCoordsYrock[0] + 100);
	ofCurveVertex(pinCoordsX[1], geoCoordsYrock[1] + 50);
	ofCurveVertex(pinCoordsX[2], geoCoordsYrock[2] + 50);
	ofCurveVertex(pinCoordsX[3], geoCoordsYrock[3] + 80);
	ofCurveVertex(pinCoordsX[4], geoCoordsYrock[4] + 120);
	ofCurveVertex(pinCoordsX[5], geoCoordsYrock[5] + 240);
	ofCurveVertex(pinCoordsX[4], geoCoordsYrock[4] + 300);
	ofCurveVertex(pinCoordsX[3], geoCoordsYrock[3] + 300);
	ofCurveVertex(pinCoordsX[2], geoCoordsYrock[2] + 300);
	ofCurveVertex(pinCoordsX[1], geoCoordsYrock[1] + 350);
	ofCurveVertex(pinCoordsX[0] - pinWidth / 2, geoCoordsYrock[0] + 300);
	ofCurveVertex(pinCoordsX[0] - pinWidth / 2, geoCoordsYrock[0] + 200);
	ofEndShape(FALSE);



	//draw rocks//
	ofSetColor(20, 20, 20, 255);
	for (int i = 0; i < 100; i++) {
		ofDrawCircle(rocksX[i], rocksY[i], 4);
	}


	geoForce.end();
	geoForce.readToPixels(forcePixels);

}

void ofApp::updatePlateForce() {

	plateForce.begin();
	ofClear(255, 255, 255, 0);


	ofSetColor(253, 253, 253, 255);
	ofDrawRectangle(pinCoordsX[0] - (pinWidth / 2), pinBaseline - (pinHeight*0.8), allPinsWidth, pinHeight*1.5);
	ofSetColor(1, 1, 1, 255);

	plateLeft.draw(ofGetWindowWidth() / 2 - plateLeftOffset, (pinBaseline - pinHeight + 50));
	plateRight.draw(ofGetWindowWidth() / 2 + plateRightOffset, (pinBaseline - pinHeight + 150));



	plateForce.end();
	plateForce.readToPixels(forcePixels);

	ofSetColor(255, 255, 255, 255);


}

void ofApp::updateHoneyForce(){

    honeyForce.begin();
    ofClear(255,255,255, 0);

    ofSetColor(90,90,90,255);
    ofDrawCircle(ofGetWindowWidth()/2,ofGetWindowHeight()/2,mouseHoverRadius*20);

    honeyForce.end();
    honeyForce.readToPixels(forcePixels);

}

void ofApp::updateTestForce() {

	testForce.begin();
	ofClear(255, 255, 255, 0);
	for (int i = 0; i < pins; i++) {
		ofFill();
		ofSetColor(testForceMapStrength[i], testForceMapStrength[i], testForceMapStrength[i], 255);
		ofDrawRectangle(pinCoordsX[i] - pinWidth / 2, (testCoordsY[i]) - 40, pinWidth, pinHeight*1.2);
	}

	testForce.end();
	ofFill();
	testForce.readToPixels(forcePixels);
}

void ofApp::updatePinForces() {
	for (int i = 0; i < pins; i++) {
		pinSimForces[i] = 255 - (forcePixels.getColor(pinCoordsX[i], pinCoordsY[i] + 5).getBrightness());

		if (appSelect == 0) {
			if (pinsTouched[i]) {
				if (mouseY > armInitCoordsY[i] && mouseY < armInitCoordsY[i] + armThresh) {
					pinSimForces[i] = pinSimForces[i] + ((mouseY - armInitCoordsY[i]) / 2);
				}
			}
		}
	}


}

//-------------------------------------------------------------- UPDATE ACTUAL PINS --------------------------------------------------------------<<

// map r1val from (r1min, r1max) to (r2min, r2max)
int ofApp::rangeMap(int r1min, int r1max, int r2min, int r2max, int r1val) {
	int val = (int)(r2min + ((double)(r2max - r2min))*(((double)(r1val - r1min)) / ((double)(r1max - r1min))));

	// Range sanity check
	if (((r2min<r2max) && (val < r2min || val > r2max)) || ((r2min>r2max) && (val < r2max || val > r2min))) {
		cout << "WTF";
	}
	return val;
}

void ofApp::updatePinHeights() {
	for (int i = 0; i < pins; i++) {
		//pinSimHeights[i] = ((pinBaseline - pinCoordsY[i]) * ((motorMaxPosition) / (pinHeight)));
		//pinSimHeights[i] = pinHeight - pinSimHeights[i];

		//1Max Pin Coordinate = 0;
		//1Min Pin Coordniate = pinHeight;
		//2Max Pin Height = motorMaxPosition;
		//2Min Pin Height = 0;


		//Map Range:       (r1val - r1min) * ((r2max - r2min)/(r1max - r1min)) + r2min;
		//pinSimHeights[i] = 0 + (int)(((double)(motorMaxPosition - 0) / ((double)0 - pinHeight)))*(pinCoordsY[i] - pinHeight);

		pinSimHeights[i] = rangeMap(pinHeight, 0, 0, motorMaxPosition, pinCoordsY[i]);//((pinCoordsY[i] - pinHeight) * ((motorMaxPosition - 0) / (0 - pinHeight)) + 0;

// happens before this  - pinCoordsY is zeroed (at pinheight)
//		if (pinSimHeights[i] < 10) {
//			cout << "WTF";
//		}
	}
}

void ofApp::updatePinCoords() {
	for (int i = 0; i < pins; i++) {
		if (pinsTouched[i]) {
			pinCoordsY[i] = rangeMap(0, motorMaxPosition, pinHeight, 0, pinsHeights[i]);
		}
	}
}

void ofApp::updatePinCoordsMouse() {

	for (int i = 0; i < pins; i++) {
		if (mouseX >(pinCoordsX[i] - mouseHoverRadius) && mouseX < (pinCoordsX[i] + mouseHoverRadius) && mouseY >(pinCoordsY[i] - mouseHoverRadius) && mouseY < (pinCoordsY[i] + mouseHoverRadius)) {
			pinsTouched[i] = true;
			pinCoordsY[i] = mouseY;
		}
		else {
			pinsTouched[i] = false;
		}
	}

}

void ofApp::setPinsNCoords() {

	//Setup Pins
	pinWidth = (ofGetWindowWidth() - (padding * 2)) / pins;
	pinHeight = pinWidth*(6.2);
	allPinsWidth = pins*pinWidth;
	pinBaseline = pinHeight + 25;


	//Setup App Initial Coordinates

	//ARM
	armInitCoordsY[0] = pinBaseline - (pinHeight*0.11);
	armInitCoordsY[1] = pinBaseline - (pinHeight*0.56);
	armInitCoordsY[2] = pinBaseline - (pinHeight*0.76);
	armInitCoordsY[3] = pinBaseline - (pinHeight*0.87);
	armInitCoordsY[4] = pinBaseline - (pinHeight*0.96);
	armInitCoordsY[5] = pinBaseline - (pinHeight * 1);
	armInitCoordsY[6] = pinBaseline - (pinHeight * 1);
	armInitCoordsY[7] = pinBaseline - (pinHeight*0.96);
	armInitCoordsY[8] = pinBaseline - (pinHeight*0.87);
	armInitCoordsY[9] = pinBaseline - (pinHeight*0.76);
	armInitCoordsY[10] = pinBaseline - (pinHeight*0.56);
	armInitCoordsY[11] = pinBaseline - (pinHeight*0.11);

	//GEO
	geoInitCoordsY[0] = pinBaseline - (pinHeight*0.8);
	geoInitCoordsY[1] = pinBaseline - (pinHeight*0.8) - 140;
	geoInitCoordsY[2] = pinBaseline - (pinHeight*0.8);
	geoInitCoordsY[3] = pinBaseline - (pinHeight*0.8);
	geoInitCoordsY[4] = pinBaseline - (pinHeight*0.5);
	geoInitCoordsY[5] = pinBaseline - (pinHeight*0.5);
	geoInitCoordsY[6] = pinBaseline - (pinHeight*0.5);
	geoInitCoordsY[7] = pinBaseline - (pinHeight*0.6);
	geoInitCoordsY[8] = pinBaseline - (pinHeight*0.6);
	geoInitCoordsY[9] = pinBaseline - (pinHeight*0.7);
	geoInitCoordsY[10] = pinBaseline - (pinHeight*0.7);
	geoInitCoordsY[11] = pinBaseline - (pinHeight*0.6);

	geoInitCoordsYrock[0] = pinBaseline - (pinHeight*0.5);
	geoInitCoordsYrock[1] = pinBaseline - (pinHeight*0.5);
	geoInitCoordsYrock[2] = pinBaseline - (pinHeight*0.5);
	geoInitCoordsYrock[3] = pinBaseline - (pinHeight*0.35);
	geoInitCoordsYrock[4] = pinBaseline - (pinHeight*0.4);
	geoInitCoordsYrock[5] = pinBaseline - (pinHeight*0.3);
	geoInitCoordsYrock[6] = pinBaseline - (pinHeight*0.3);
	geoInitCoordsYrock[7] = pinBaseline - (pinHeight*0.35);
	geoInitCoordsYrock[8] = pinBaseline - (pinHeight*0.4);
	geoInitCoordsYrock[9] = pinBaseline - (pinHeight*0.4);
	geoInitCoordsYrock[10] = pinBaseline - (pinHeight*0.35);
	geoInitCoordsYrock[11] = pinBaseline - (pinHeight*0.3);

	//Setup App Live Coordinates
	for (int i = 0; i < pins; i++) {
		//pinCoordsX[i] = (padding + (pinWidth / 2) + (pinWidth*i));
		pinCoordsX[i] = (padding + (pinWidth / 2) + (pinWidth*i));


		armCoordsY1[i] = armInitCoordsY[i];
		geoCoordsY[i] = geoInitCoordsY[i];
		geoCoordsYrock[i] = geoInitCoordsYrock[i];

		testInitCoordsY[i] = pinBaseline - (pinHeight*0.7);
		testCoordsY[i] = testInitCoordsY[i];
		testForceMapStrength[i] = (255 / pins)*i;

		//PLATES
		plateInitCoordsY[i] = pinBaseline - (pinHeight*0.7);

		//HONEY
		honeyInitCoordsY[i] = pinBaseline-(pinHeight*0.4);

		pinCoordsY[i] = testCoordsY[i];

	}



	//Draw Guide Lines
	pinsMin.clear();
	pinsMax.clear();
	pinsMin.addVertex(0, pinBaseline);
	pinsMin.addVertex(ofGetWindowWidth(), pinBaseline);
	pinsMax.addVertex(0, pinBaseline - pinHeight);
	pinsMax.addVertex(ofGetWindowWidth(), pinBaseline - pinHeight);

	//Update Arm Forces
	armForceMapY[1] = pinBaseline;
	armForceMapY[0] = pinBaseline + 10;
}

void ofApp::drawInfo() {
	//Draw Pins and Points
	if (showTheInfo) {

		//Draw Pins
		ofNoFill();
		ofSetColor(225, 255, 255, 255);
		ofSetLineWidth(5);
		//    ofDrawCircle(ofGetWindowWidth()/2,ofGetWindowHeight()/2,375);
		for (int i = 0; i < pins; i++) {
			ofDrawRectangle(pinCoordsX[i] - pinWidth / 2, (pinCoordsY[i]), pinWidth, pinHeight);
		}

		//Draw Pin Min and Max
		ofNoFill();
		ofSetColor(0, 0, 255, 255);
		ofSetLineWidth(5);
		pinsMin.draw();
		pinsMax.draw();

		ofSetColor(255, 255, 255, 255);

		if (mouseControl) {
			ofDrawBitmapString("MOUSE CONTROL (M to toggle)", 100, 100);
		} else {
			ofDrawBitmapString("INFORCE CONTROL (M to toggle)", 100, 100);
		}

		//Draw Forces Info
		ofFill();
		for (int i = 0; i < pins; i++) {
			ofSetColor(225, 0, 0, pinSimForces[i]);
			ofDrawCircle(pinCoordsX[i], pinCoordsY[i], mouseHoverRadius / 2);
			ofSetColor(225, 255, 255, 255);
			ofDrawRectangle(pinCoordsX[i] - (pinWidth / 2), pinCoordsY[i] + 20, pinWidth, 40);
			ofSetColor(225, 255, 255, 255);
			ofDrawBitmapString(pinSimForces[i], pinCoordsX[i] - 12, pinCoordsY[i] + 5);

			ofSetColor(0, 0, 0, 255);
			ofDrawBitmapString("To Pins", pinCoordsX[i] - 26, pinCoordsY[i] + 37);
			ofDrawBitmapString(pinSimHeights[i], pinCoordsX[i] - 17, pinCoordsY[i] + 55);

			ofSetColor(225, 255, 255, 255);
			ofDrawBitmapString("YCoords", pinCoordsX[i] - 28, pinCoordsY[i] + 75);
			ofDrawBitmapString(pinCoordsY[i], pinCoordsX[i] - 15, pinCoordsY[i] + 85);
			ofSetColor(0, 255, 0, 255);
			ofFill();
			if (pinsTouched[i]) {
				ofDrawCircle(pinCoordsX[i], pinCoordsY[i]+120, mouseHoverRadius / 2);
				ofSetColor(0, 0, 0, 255);

				ofDrawBitmapString(pinPressuresBuffer[i], pinCoordsX[i] - 15, pinCoordsY[i] + 120);
			}
		}

		ofSetColor(225, 225, 225, 255);


	}
	else {
		//Hide Below pinBaseline
		//        ofSetColor(0, 0, 0,255);
		//        ofFill();
		//        ofDrawRectangle(0,pinBaseline,ofGetWindowWidth(),ofGetWindowHeight()-pinBaseline);
	}

	if (showAssist) {
		ofFill();
		ofSetColor(225, 0, 0, 255);
		ofDrawCircle(0, 0, mouseHoverRadius);
		ofDrawCircle(ofGetWindowWidth(), 0, mouseHoverRadius);
		ofDrawCircle(ofGetWindowWidth(), ofGetWindowHeight(), mouseHoverRadius);
		ofDrawCircle(0, ofGetWindowHeight(), mouseHoverRadius);

		ofSetLineWidth(2);
		ofNoFill();
		ofSetColor(225, 0, 0, 255);
		lineVert.draw();
		lineHorz.draw();
	}
}

//-------------------------------------------------------------- ARM SIMULATION --------------------------------------------------------------<<
void ofApp::armApp() {


	//Draw Skin Layer One
	ofFill();
	ofSetColor(251, 243, 145, 255);

	ofSetPolyMode(OF_POLY_WINDING_NONZERO);
	ofBeginShape();

	ofCurveVertex(pinCoordsX[0], armCoordsY1[0]);


	// update Pins // REFACTOR!
	for (int i = 0; i < pins; i++) {

		ofCurveVertex(pinCoordsX[i], armCoordsY1[i]);

		if (pinsTouched[i]) {

			if (pinCoordsY[i] > armInitCoordsY[i] && pinCoordsY[i] < armInitCoordsY[i] + armThresh) {
				armCoordsY1[i] = mouseY;
			}

			//Update Pins
			if (pinCoordsY[i] > armInitCoordsY[i]) {
				pinCoordsY[i] = mouseY;
			}

		}
		else {

			//Skin Return at Skin Elasticity
			if (armCoordsY1[i] > armInitCoordsY[i]) {
				armCoordsY1[i] = armCoordsY1[i] - skinElasticity;
				if (armCoordsY1[i]<armInitCoordsY[i]) {
					armCoordsY1[i] = armInitCoordsY[i];
				}
			}

			//Pins Return at Pin Elasticity
			if (pinCoordsY[i]>armInitCoordsY[i]) {
				pinCoordsY[i] = pinCoordsY[i] - pinElasticity;
				if (pinCoordsY[i]<armInitCoordsY[i])
					pinCoordsY[i] = armInitCoordsY[i];
			}

		}
	}

	for (int i = pins - 2; i > -1; i--) {
		ofCurveVertex(pinCoordsX[i], (pinBaseline - armInitCoordsY[i] + pinHeight));
	}



	ofEndShape();



	//Draw the MEAAT
	ofSetColor(255, 255, 255, 255);
	ofNoFill();
	armImage.resize((pinWidth*pins) - 150, (pinWidth*pins) - 150);
	armImage.setAnchorPoint(armImage.getWidth() / 2, armImage.getHeight() / 2);
	armImage.draw(ofGetWindowWidth() / 2, pinBaseline);


	//ReturnPins
	for (int i = 0; i < pins; i++) {
		if (pinCoordsY[i] > armInitCoordsY[i]) {
			if (pinsTouched[i] == false) {
				pinCoordsY[i] = pinCoordsY[i] - pinElasticity;
				if (pinCoordsY[i] < armInitCoordsY[i]) {
					pinCoordsY[i] = armInitCoordsY[i];
				}
			}
		}
		if (pinCoordsY[i] < armInitCoordsY[i]) {
			if (pinsTouched[i] == false) {
				pinCoordsY[i]++;
			}
		}

	}

	//Draw the Force
	if (showTheForce) {
		armForce.draw(0, 0);
	}
}

//-------------------------------------------------------------- GEO SIMULATION --------------------------------------------------------------<<
void ofApp::geoApp() {

	for (int i = 0; i < pins; i++) {
		if (pinsTouched[i] && pinCoordsY[i] < geoInitCoordsY[i] + geoThresh) {
			geoCoordsY[i] = pinCoordsY[i];
		}

		if (!pinsTouched[i]) {
			if (pinCoordsY[i]>geoCoordsY[i]) {
				pinCoordsY[i] = pinCoordsY[i] - pinElasticity;
				if (pinCoordsY[i]<geoCoordsY[i])
					pinCoordsY[i] = geoCoordsY[i];
			}
		}
	}


	//Draw SCENE

	//Draw Ocean
	ofSetColor(0, 150, 240, 255);
	ofFill();
	ofDrawRectangle((ofGetWindowWidth() / 2) - (allPinsWidth / 2), geoInitCoordsY[0] + 30, allPinsWidth - 3, pinHeight);

	//Draw Oil Thing
	ofSetColor(255, 255, 255, 255);
	//    rig.resize(pinWidth, );
	rig.setAnchorPoint(rig.getWidth() / 2, 0);
	rig.draw(pinCoordsX[1], geoCoordsY[1] + 20);

	//Draw Earth Layer
	ofSetColor(123, 92, 64, 255);
	ofFill();
	ofBeginShape();
	for (int i = 0; i < pins; i++) {

		if (i == 0) {
			ofCurveVertex((pinCoordsX[i] - (pinWidth / 2)), geoCoordsY[i]);
		}
		else if (i == (pins - 1)) {
			ofCurveVertex((pinCoordsX[i] + (pinWidth / 2)), geoCoordsY[i]);
		}
		else if (i == 1) {
			ofCurveVertex((pinCoordsX[i] + (pinWidth / 2)), geoCoordsY[i] + 140);
		}
		else {
			ofCurveVertex(pinCoordsX[i], geoCoordsY[i]);
		}

	}
	ofCurveVertex(pinCoordsX[11] + (pinWidth / 2), geoCoordsY[11] + 2);
	ofVertex(pinCoordsX[11] + (pinWidth / 2), ofGetWindowHeight());
	ofVertex(pinCoordsX[0] - (pinWidth / 2), ofGetWindowHeight());
	ofVertex(pinCoordsX[0] - (pinWidth / 2), geoCoordsY[0]);
	ofEndShape(FALSE);


	//Draw Grass Layer
	geoGrassline.clear();
	geoGrassline.addVertex((pinCoordsX[0] - (pinWidth / 2)), geoCoordsY[0] + 15);
	for (int i = 0; i < pins; i++) {

		if (i == (pins - 1)) {
			geoGrassline.curveTo((pinCoordsX[i] + (pinWidth / 2)), geoCoordsY[i] + 15);
		}
		else if (i == 1) {
			geoGrassline.curveTo((pinCoordsX[i] + (pinWidth / 2)), geoCoordsY[i] + 155);
		}
		else {
			geoGrassline.curveTo(pinCoordsX[i], geoCoordsY[i] + 15);
		}

	}
	geoGrassline.addVertex(pinCoordsX[11] + (pinWidth / 2), geoCoordsY[11] + 15);
	ofSetColor(0, 240, 90, 255);
	glEnable(GL_POINT_SMOOTH);
	glLineWidth(30);
	ofSetPolyMode(OF_POLY_WINDING_NONZERO);

	//  ofSetLineWidth(30);
	ofNoFill();
	geoGrassline.draw();


	//Draw Sediment Layer
	ofSetColor(112, 70, 40, 255);
	ofFill();
	ofBeginShape();
	for (int i = 0; i < pins; i++) {

		if (i == 0) {
			ofCurveVertex((pinCoordsX[i] - (pinWidth / 2)), geoCoordsYrock[i]);
		}
		else if (i == (pins - 1)) {
			ofCurveVertex((pinCoordsX[i] + (pinWidth / 2)), geoCoordsYrock[i]);
		}
		else {
			ofCurveVertex(pinCoordsX[i], geoCoordsYrock[i]);
		}

	}
	ofCurveVertex(pinCoordsX[11] + (pinWidth / 2), geoCoordsYrock[11] + 2);
	ofVertex(pinCoordsX[11] + (pinWidth / 2), ofGetWindowHeight());
	ofVertex(pinCoordsX[0] - (pinWidth / 2), ofGetWindowHeight());
	ofVertex(pinCoordsX[0] - (pinWidth / 2), geoCoordsYrock[0]);
	ofEndShape(FALSE);


	//Draw Oil
	ofSetColor(30, 30, 30, 255);
	ofFill();
	ofBeginShape();
	ofCurveVertex(pinCoordsX[0] - pinWidth / 2, geoCoordsYrock[0] + 100);
	ofCurveVertex(pinCoordsX[0] - pinWidth / 2, geoCoordsYrock[0] + 100);
	ofCurveVertex(pinCoordsX[1], geoCoordsYrock[1] + 50);
	ofCurveVertex(pinCoordsX[2], geoCoordsYrock[2] + 50);
	ofCurveVertex(pinCoordsX[3], geoCoordsYrock[3] + 80);
	ofCurveVertex(pinCoordsX[4], geoCoordsYrock[4] + 120);
	ofCurveVertex(pinCoordsX[5], geoCoordsYrock[5] + 240);
	ofCurveVertex(pinCoordsX[4], geoCoordsYrock[4] + 300);
	ofCurveVertex(pinCoordsX[3], geoCoordsYrock[3] + 300);
	ofCurveVertex(pinCoordsX[2], geoCoordsYrock[2] + 300);
	ofCurveVertex(pinCoordsX[1], geoCoordsYrock[1] + 350);
	ofCurveVertex(pinCoordsX[0] - pinWidth / 2, geoCoordsYrock[0] + 300);
	ofCurveVertex(pinCoordsX[0] - pinWidth / 2, geoCoordsYrock[0] + 200);
	ofEndShape(FALSE);

	//Draw Oil line
	ofSetColor(223, 17, 142, 255);
	ofSetLineWidth(5);
	ofDrawLine(pinCoordsX[1] + 1, geoCoordsY[1] + 30, pinCoordsX[1] + 1, pinBaseline - 150);


	ofSetColor(255, 255, 255, 255);

	//Draw the Force
	if (showTheForce) {
		geoForce.draw(0, 0);
	}


}

//-------------------------------------------------------------- TECTONIC SIMULATION --------------------------------------------------------------<<
void ofApp::plateApp() {



	//Draw the Water
	ofSetColor(255, 255, 255, 255);
	ofNoFill();
	water.resize((allPinsWidth), (allPinsWidth));
	water.setAnchorPoint(water.getWidth() / 2, 0);
	water.draw(ofGetWindowWidth() / 2, pinBaseline - (pinHeight*0.7));

	//Draw the Magma
	magma1.resize((allPinsWidth), (allPinsWidth));
	magma1.setAnchorPoint(magma1.getWidth() / 2, 0);
	magma1.draw(ofGetWindowWidth() / 2, (pinBaseline - pinHeight + 35));
	if (magma2Complete == false) {
		magma2Opacity++;
		if (magma2Opacity == 255) {
			magma2Complete = true;
		}
	}
	else {
		magma2Opacity--;
		if (magma2Opacity == 0) {
			magma2Complete = false;
		}
	}
	ofSetColor(255, 255, 255, magma2Opacity);
	magma2.resize((allPinsWidth), (allPinsWidth));
	magma2.setAnchorPoint(magma2.getWidth() / 2, 0);
	magma2.draw(ofGetWindowWidth() / 2, (pinBaseline - pinHeight + 35));


	//Move the Plates
	if (faultVal < 145) {
		if (pinSimForces[8] > 253 || pinSimForces[9] > 253 || pinSimForces[10] > 253 || pinSimForces[11] > 253) {
			plateRightOffset--;
			faultVal++;
			volcanoScale = volcanoScale + 0.07;
			volcanoScaleX = volcanoScaleX + 0.04;
			tunnelScale = tunnelScale + 0.07;
			sound_lava.setVolume(1.0f);
		}
		else if (pinSimForces[0] > 253 || pinSimForces[1] > 253 || pinSimForces[2] > 253 || pinSimForces[3] > 253 || pinSimForces[4] > 253 || pinSimForces[5] > 253 || pinSimForces[6] > 253 || pinSimForces[7] > 253) {
			plateLeftOffset--;
			faultVal++;
			volcanoScale = volcanoScale + 0.07;
			volcanoScaleX = volcanoScaleX + 0.04;
			tunnelScale = tunnelScale + 0.07;
			sound_lava.setVolume(1.0f);
		}
		else {
			sound_lava.setVolume(0.1f);
		}
	}
	else {
		volcano = true;
		//volcano go BOOM FUCKITY BOOM
	}

	//Draw the Volcano
	ofSetColor(142, 113, 93, 255);
	ofFill();
	ofBeginShape();
	int magmaY = (pinBaseline - pinHeight + 35);
	ofCurveVertex(pinCoordsX[2] - pinWidth - 10 - volcanoScaleX, magmaY + 200);
	ofCurveVertex(pinCoordsX[2] - pinWidth - volcanoScaleX, magmaY + 200 - (volcanoScaleX / 2));
	ofCurveVertex(pinCoordsX[2], magmaY + 130 - volcanoScale);
	ofCurveVertex(pinCoordsX[2] + pinWidth + volcanoScaleX, magmaY + 200 - (volcanoScaleX / 2));
	ofCurveVertex(pinCoordsX[2] + pinWidth + 10 + volcanoScaleX, magmaY + 200);
	ofCurveVertex(pinCoordsX[2] - pinWidth - 10, magmaY + 200);
	ofEndShape();


	//Draw the Plates
	ofSetColor(255, 255, 255, 255);
	plateLeft.resize((allPinsWidth), (allPinsWidth));
	plateLeft.setAnchorPoint(plateLeft.getWidth() / 2, 0);
	plateLeft.draw(ofGetWindowWidth() / 2 - plateLeftOffset, (pinBaseline - pinHeight + 50));
	plateRight.resize((allPinsWidth), (allPinsWidth));
	plateRight.setAnchorPoint(plateRight.getWidth() / 2, 0);
	plateRight.draw(ofGetWindowWidth() / 2 + plateRightOffset, (pinBaseline - pinHeight + 170));


	//Draw Magma Tunnel
	ofSetColor(196, 54, 3, 255);
	ofBeginShape();
	ofCurveVertex(pinCoordsX[2] - 11 - (tunnelScale), magmaY + 315);
	ofCurveVertex(pinCoordsX[2] - 10 - (tunnelScale), magmaY + 315);
	ofCurveVertex(pinCoordsX[2], magmaY + 260 - (tunnelScale * 15));
	ofCurveVertex(pinCoordsX[2] + 10 + (tunnelScale), magmaY + 318);
	ofCurveVertex(pinCoordsX[2] + 11 + (tunnelScale), magmaY + 318);
	ofEndShape();


	if (volcano) {
		if (!sound_volcano.isPlaying()) {
			sound_volcano.play();
		}

		if (volcanoScale < 120) {
			volcanoScale = volcanoScale + 6;
			tunnelScale = tunnelScale + 0.38;
			pinCoordsY[2] = pinCoordsY[2] - volcanoScale / 13;
			pinCoordsY[1] = pinCoordsY[1] - volcanoScale / 26;
			pinCoordsY[3] = pinCoordsY[3] - volcanoScale / 26;

		}

		if (volcanoScale > 99) {
			if (volcanoScaleX < 40) {
				volcanoScale = volcanoScale + 0.05;
				volcanoScaleX = volcanoScaleX + 0.14;
			}
		}

		if (volcanoScaleX > 39.9) {
			volcanoWaiting = true;
			volcanoTopCoord = pinCoordsY[2];
			volcano = false;
		}

	}

	if (volcanoWaiting) {

		//        if (pinCoordsY[2] > 100){
		if (plateRightOffset<200) {
			plateRightOffset++;
		}
		else {
			plateRightOffset = 200;
		}
		if (plateLeftOffset<200) {
			plateLeftOffset++;
		}
		else {
			plateLeftOffset = 200;
		}
		if (faultVal>0) {
			faultVal--;
		}
		else {
			faultVal = 0;
		}
		if (volcanoScale>0) {
			volcanoScale = volcanoScale - 1;
		}
		else {
			volcanoScale = 0;
		}
		if (volcanoScaleX>0) {
			volcanoScaleX = volcanoScaleX - 1;
		}
		else {
			volcanoScaleX = 0;
		}
		if (tunnelScale>0) {
			tunnelScale = tunnelScale - 1;
		}
		else {
			tunnelScale = 0;
		}

		if (pinCoordsY[1] < plateInitCoordsY[2]) {
			pinCoordsY[1]++;
		}
		if (pinCoordsY[2] < plateInitCoordsY[2]) {
			pinCoordsY[2]++;
		}
		if (pinCoordsY[3] < plateInitCoordsY[2]) {
			pinCoordsY[3]++;
		}

		if (plateRightOffset == 200 && plateLeftOffset == 200 && faultVal == 0 && volcanoScale == 0 && volcanoScaleX == 0 && tunnelScale == 0) {
			volcanoWaiting = false;

		}
		//        }


	}


	//ReturnPins
	if (!volcano) {
		for (int i = 0; i < pins; i++) {
			if (pinCoordsY[i] > plateInitCoordsY[i]) {
				if (pinsTouched[i] == false) {
					pinCoordsY[i] = pinCoordsY[i] - pinElasticity;
					if (pinCoordsY[i] < plateInitCoordsY[i]) {
						pinCoordsY[i] = plateInitCoordsY[i];
					}
				}
			}
			if (pinCoordsY[i] < plateInitCoordsY[i]) {
				if (pinsTouched[i] == false) {
					pinCoordsY[i]++;
				}
			}
		}
	}


	ofSetColor(255, 255, 255, 255);

	//Draw the Force
	if (showTheForce) {
		plateForce.draw(0, 0);
	}

	ofSetColor(0, 0, 0, 255);
	ofFill();
	ofDrawRectangle(0, 0, padding, ofGetWindowHeight());
	ofDrawRectangle(padding + allPinsWidth, 0, padding, ofGetWindowHeight());

}


//-------------------------------------------------------------- TEST SIMULATION --------------------------------------------------------------<<
void ofApp::simApp() {


	for (int i = 0; i < pins; i++) {

		if (pinsTouched[i]) {
			testCoordsY[i] = pinCoordsY[i];
		}

		ofSetColor(255 - (255 / pins)*i, 0, (255 / pins)*i, 255);
		ofFill();
		ofDrawRectangle(pinCoordsX[i] - pinWidth / 2, (testCoordsY[i]), pinWidth, pinHeight * 2);

	}

	ofSetColor(255, 255, 255, 255);


	//ReturnPins
	for (int i = 0; i < pins; i++) {
		if (pinCoordsY[i] > testInitCoordsY[i]) {
			if (pinsTouched[i] == false) {
				pinCoordsY[i] = pinCoordsY[i] - pinElasticity/4;
				if (pinCoordsY[i] < testInitCoordsY[i]) {
					pinCoordsY[i] = testInitCoordsY[i];
				}
			}
		}
		if (pinCoordsY[i] < testInitCoordsY[i]) {
			if (pinsTouched[i] == false) {
				pinCoordsY[i]= pinCoordsY[i] + pinElasticity/4;
			}
		}

		testCoordsY[i] = pinCoordsY[i];

	}

	//Draw the Force
	if (showTheForce) {
		testForce.draw(0, 0);
	}



}

//-------------------------------------------------------------- HONEY APP --------------------------------------------------------------<<


void ofApp::honeyApp(){


    for (int i = 0; i < pins; i++) {
        //Yellow Color
        ofSetColor(253, 211, 120, 255);
        ofFill();
        ofDrawRectangle(pinCoordsX[i]-pinWidth/2,(pinCoordsY[i]),pinWidth,pinHeight*10);
    }

    for (int i = 0; i < pins; i++) {

        if(pinsTouched[i] == true){
            int slope = 1.2;

                for (int n = i-(honeyViscosity/2); n < i+(honeyViscosity/2)+1; n++) {
                    if(i != n && n >= 0 && n < pins ){
//                            pinCoordsY[n] = pinCoordsY[n] - ((pinCoordsY[n] - pinCoordsY[i]) / pow (abs(i-n), 3.0));
						int newPos = honeyInitCoordsY[i] + ((pinCoordsY[i] - honeyInitCoordsY[i]) / pow(2 * abs(i - n), slope));

						// Only move the pin to this new position if it would be farhter from the initial position than it already is.
						if (abs(newPos - honeyInitCoordsY[i]) > abs(pinCoordsY[n] - honeyInitCoordsY[i])) {
							pinCoordsY[n] = newPos;
						}
                    }
                }
        }


    }



    ofSetColor(255,255,255,255);


    //ReturnPins
    for (int i = 0; i < pins; i++) {
        if(pinCoordsY[i] < honeyInitCoordsY[i]){
            if(pinsTouched[i] == false){
                pinCoordsY[i] = pinCoordsY[i]+(honeyViscosity*3);
                if (pinCoordsY[i] > honeyInitCoordsY[i]){
                    pinCoordsY[i] = honeyInitCoordsY[i];
                }
            }
        }
        if(pinCoordsY[i] > testInitCoordsY[i]){
            if(pinsTouched[i] == false){
                pinCoordsY[i]= pinCoordsY[i]-honeyViscosity;
            }
        }


    }

    //Draw the Force
    if(showTheForce){
        honeyForce.draw(0,0);
    }



}

//-------------------------------------------------------------- WAVE APP --------------------------------------------------------------<<

void ofApp::waveApp(){
	static const int SIN_MIN = pinBaseline - 50;
	static const int SIN_MAX = pinBaseline - pinHeight + 50;//; // pinBaseline;
    static const int SIN_AMPLITUDE = (SIN_MAX - SIN_MIN)/2;
    static const int SIN_OFFSET = SIN_AMPLITUDE / 2 -150;
    static const float multiplier = PI * 2.0 / 11.0;

    static double x = 0.0;
    static double height = 0.0;

    for (int i = 0; i < pins; i++) {
        ofSetColor(255-(255/pins)*i, 0, (255/pins)*i, 255);

        ofFill();
        pinSimForces[i] = 254;
        x += 0.005;
        //cout << "\n";
            height = SIN_MIN + SIN_OFFSET + SIN_AMPLITUDE * sin((i+x)*multiplier);
            //cout << height << "\t";

            pinCoordsY[i] = int(height);
        ofDrawRectangle(pinCoordsX[i]-pinWidth/2,(pinCoordsY[i]),pinWidth,pinHeight);

    }
}



//-------------------------------------------------------------- HELPER FUNCTIONS --------------------------------------------------------------<<

bool ofApp::overCircle(int x, int y, int diameter) {
	float disX = x - mouseX;
	float disY = y - mouseY;
	if (sqrt(pow(5.0, disX) + pow(5.0, disY)) < diameter / 2) {
		return true;
	}
	else {
		return false;
	}
}

bool ofApp::overSquare(int x, int y, int width) {
	if (mouseX > (x - width) && mouseX < (x + width) && mouseY >(y - width) && mouseY < (y + width)) {
		return true;
	}
	else {
		return false;
	}
}

//void ofApp::animateTo(int current, int final, int elasticFactor){
//
//    if (current > final) {
//        current = current - elasticFactor;
//        if (current< final){
//            current = final;
//        }
//    }
//
////    if(current!=final){
////        if (current>final) {
////            current = current-elasticFactor;
////            if (current<final)
////                current = final;
////        } else {
////            current = current+elasticFactor;
////            if (current>final)
////                current = final;
////
////        }
////    }
//
//
//}


//-------------------------------------------------------------- KEY PRESSES --------------------------------------------------------------<<
void ofApp::keyPressed(int key) {

	switch (key) {
	case '/':
		showTheInfo = !showTheInfo;
		break;
	case '*':
		showTheForce = !showTheForce;
		break;

	case 'i':
		showTheInfo = !showTheInfo;
		break;
	case 'm':
		mouseControl = !mouseControl;
		break;
	case 'p':
		showAssist = !showAssist;
		break;
	case 'f':
		showTheForce = !showTheForce;
		break;
	case '1':
		sound_lava.stop();
		appSelect = 0;
		for (int i = 0; i < pins; i++) {
			pinCoordsY[i] = armInitCoordsY[i];
		}
		inForce->enableForceControl();
		break;
	case '2':
		sound_lava.stop();

		appSelect = 1;
		for (int i = 0; i < pins; i++) {
			pinCoordsY[i] = geoInitCoordsY[i];
		}
		inForce->enableForceControl();
		break;
	case '3':
		sound_lava.stop();

		appSelect = 2;
		for (int i = 0; i < pins; i++) {
			pinCoordsY[i] = testInitCoordsY[i];
		}
		inForce->enableForceControl();
		break;
	case '4':
		appSelect = 3;
		sound_lava.play();
		sound_lava.setVolume(0.1f);
		for (int i = 0; i < pins; i++) {
			pinCoordsY[i] = plateInitCoordsY[i];
		}
		inForce->enableForceControl();
		break;
	case '5':
		sound_lava.stop();
		appSelect = 4;
		for(int i = 0; i < pins; i++){
			pinCoordsY[i] = honeyInitCoordsY[i];
		}
		inForce->enableForceControl();
		break;
	case '6':
	    sound_lava.stop();
	    appSelect = 5;
		inForce->dissableForceControl();
	    break;
	case OF_KEY_RIGHT:
		projectionX++;
		break;
	case OF_KEY_LEFT:
		projectionX--;
		break;
	case OF_KEY_UP:
		projectionY--;
		break;
	case OF_KEY_DOWN:
		projectionY++;
		break;
	case 'x':
		projectionX = 0;
		projectionY = 0;
		break;
	case '-':
		padding++;
		pinBaseline--;
		setPinsNCoords();
		cout << "PADDING:" << padding;
		cout << "pinBaseline:" << pinBaseline << "\n";
		break;
	case '=':
		padding--;
		pinBaseline++;
		setPinsNCoords();
		cout << "PADDING:" << padding;
		cout << "pinBaseline:" << pinBaseline << "\n";
		break;
	case '0':
		padding = 200;
		setPinsNCoords();
		break;
	case '[':
		pinBaseline++;
		setPinsNCoords();
		cout << "pinBaseline:" << pinBaseline << "\n";
		break;
	case ']':
		pinBaseline--;
		setPinsNCoords();
		cout << "pinBaseline:" << pinBaseline << "\n";
		break;
	case OF_KEY_ESC:
		exit();
		break;
	}
}

//-------------------------------------------------------------- OF BULLSHIT --------------------------------------------------------------<<

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
