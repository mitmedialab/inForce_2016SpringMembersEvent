#include "ofMain.h"
#include "ofApp.h"

#include "PCBUSB.h"
#include <PCBUSB.h>

#include "ofAppGlutWindow.h"
#include "ofAppGLFWWindow.h"

#include <signal.h>
#include <chrono>

static volatile int keepRunning = 1;

/*TODO
X-Verify everything works with acceptible speed over usb hub
X-Figure out wierd readings query bug
-Tune PIDs
-Implement self-calibration routine to generate force-offset lookup tables
-use lookup tables for feedforward control
-Implement better edge-case behaviors for force-step-changes and activation/deactivation
-TEST
*/

////////////////////////////////////////// QUITTING AND MAIN //////////////////////////////////////////
ofBaseApp* app;
void intHandler(int dummy) {
	app->exit();
}

int main() {

	////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////// GRAPHICS /////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////

	//    
	//    ofAppGLFWWindow window;
	//    //win.set eNumSamples(8);
	//    window.setMultiDisplayFullscreen(true); //this makes the fullscreen window span across all your monitors
	//    ofSetupOpenGL(&window, 1920, 1080, OF_FULLSCREEN);			// <-------- setup the GL context
	//    ofSetWindowPosition(0, -1080); 	//	for positioning to second screen:

	//	ofSetupOpenGL( 1280,800, OF_FULLSCREEN);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:

	// Get screen widths and heights from Quartz Services
	// See https://developer.apple.com/library/mac/documentation/GraphicsImaging/Reference/Quartz_Services_Ref/index.html

#ifdef APPLE
	CGDisplayCount displayCount;
	CGDirectDisplayID displays[32];

	// Grab the active displays
	CGGetActiveDisplayList(32, displays, &displayCount);
	int numDisplays = displayCount;

	// If two displays present, use the 2nd one. If one, use the first.
	int projector = numDisplays - 1;
	int desktop = numDisplays;

	int displayHeightProjector = CGDisplayPixelsHigh(displays[projector]);
	int displayWidthProjector = CGDisplayPixelsWide(displays[projector]);

	int displayHeightDesktop = CGDisplayPixelsHigh(displays[desktop]);
	int displayWidthDesktop = CGDisplayPixelsWide(displays[desktop]);
	/* /////////////// this is a bit slow to do  /////////////
	// move 2nd display to right of primary display and align display tops
	if(numDisplays > 0){
	CGDisplayConfigRef displayConfig;
	CGBeginDisplayConfiguration ( &displayConfig );
	CGConfigureDisplayOrigin ( displayConfig, displays[1], CGDisplayPixelsWide(displays[0]), 0 );
	CGCompleteDisplayConfiguration ( displayConfig, kCGConfigureForAppOnly );
	}
	*/
	//* //////// instead let's just moving our window to wherever our display is living:

	CGRect displayBoundsProjector = CGDisplayBounds(displays[projector]);
	CGRect displayBoundsDesktop = CGDisplayBounds(displays[desktop]);

	ofSetupOpenGL(displayWidthProjector, displayHeightProjector, OF_FULLSCREEN);            // <-------- setup the GL context

																							// that OF_FULLSCREEN makes the window as big as the primary display, but we want it to be as big as whichever we're using
	ofSetWindowShape(displayWidthProjector + displayWidthDesktop, displayHeightProjector);
	// move onto our display.
	ofSetWindowPosition(displayBoundsProjector.origin.x, displayBoundsProjector.origin.y);

	// print display info.
	cout << numDisplays << " display(s) detected." << endl << "Using Projector as Display: " << projector << " (" << displayWidthProjector << "x" << displayHeightProjector << ")." << endl;
#endif

	////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////// ORIGINAL /////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////
	signal(SIGINT, intHandler); // capture signal interruptions (Ctrl+C) which end the program, send them to intHandler()

	ofLog(OF_LOG_NOTICE, "Starting GUI...");

	// Original
	//ofSetupOpenGL(1280, 800, OF_FULLSCREEN);			// <-------- setup the GL context
	ofSetupOpenGL(1920, 1080, OF_WINDOW);	// <-------- setup the GL context
											// this kicks off the running of my app
											// can be OF_WINDOW or OF_FULLSCREEN
											// pass in width and height too:
	app = new ofApp();
	ofRunApp(app);

	ofLog(OF_LOG_NOTICE, "GOODBYE!");
}