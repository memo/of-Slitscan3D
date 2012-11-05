#include "testApp.h"
#include "MSASpaceTime.h"

float nearThreshold = 0;
float farThreshold = 3000;

float webcamNear = 1000;
float webcamFar = 2000;

int pixelStep = 1;          // how many pixels to step through the depth map when iterating
int numScanFrames = 240;    // duration (in frames) for full scan

// physical boundaries of space time continuum
ofVec3f spaceBoundaryMin    = ofVec3f(-400, -400, 400);
ofVec3f spaceBoundaryMax    = ofVec3f(400, 400, farThreshold);

// spatial resolution for space time continuum
ofVec3f spaceNumCells;


bool doSaveMesh = false;
bool doPause = false;
bool doDrawPointCloud = true;
bool doSlitScan = true;
bool doDebugInfo = false;

bool usingKinect;   // using kinect or webcam

int gradientMode = 0;
string gradientModeStr = "";

int kinectAngle;
float inputWidth, inputHeight;

msa::SpaceTime<ofMesh> spaceTime;   // space time continuum

ofMesh mesh;    // final mesh


//--------------------------------------------------------------
void fillMeshFromKinect(ofMesh &m, ofxKinect &kinect) {
    ofVec3f maxP(-10000, -10000, -10000);
    ofVec3f minP( 10000,  10000,  10000);
    for(int j=0; j<kinect.getHeight(); j += pixelStep) {
        for(int i=0; i<kinect.getWidth(); i += pixelStep) {
            ofVec3f p = kinect.getWorldCoordinateAt(i, j);
            if(ofInRange(p.z, nearThreshold, farThreshold) && kinect.getDistanceAt(i, j) > 0) {
                m.addVertex(p);
                m.addColor(kinect.getColorAt(i, j));
                
                if(doDebugInfo) {
                    if(p.x < minP.x) minP.x = p.x;
                    if(p.x > maxP.x) maxP.x = p.x;
                    
                    if(p.y < minP.y) minP.y = p.y;
                    if(p.y > maxP.y) maxP.y = p.y;
                    
                    if(p.z < minP.z) minP.z = p.z;
                    if(p.z > maxP.z) maxP.z = p.z;
                }
                
            }
        }
    }
    if(doDebugInfo) printf("Boundaries: (%f, %f, %f) - (%f, %f, %f)\n", minP.x, minP.y, minP.z, maxP.x, maxP.y, maxP.z);
}


//--------------------------------------------------------------
// if kinect isn't available, just use webcam for testing (use brightness for depth value)
void fillMeshFromPixels(ofMesh &m, ofPixelsRef pixelsRef) {
    ofVec3f maxP(-10000, -10000, -10000);
    ofVec3f minP( 10000,  10000,  10000);
    for(int j=0; j<pixelsRef.getHeight(); j += pixelStep) {
        for(int i=0; i<pixelsRef.getWidth(); i += pixelStep) {
            ofColor c = pixelsRef.getColor(i, j);
            ofVec3f p;
            p.x = ofMap(i, 0, pixelsRef.getWidth(), spaceBoundaryMin.x, spaceBoundaryMax.x);
            p.y = ofMap(j, 0, pixelsRef.getHeight(), spaceBoundaryMin.y, spaceBoundaryMax.y);
            p.z = ofMap(c.getBrightness(), 255, 0, webcamNear, webcamFar);
            if(ofInRange(p.z, nearThreshold, farThreshold)) {
                m.addVertex(p);
                m.addColor(c);
                
                if(doDebugInfo) {
                    if(p.x < minP.x) minP.x = p.x;
                    if(p.x > maxP.x) maxP.x = p.x;
                    
                    if(p.y < minP.y) minP.y = p.y;
                    if(p.y > maxP.y) maxP.y = p.y;
                    
                    if(p.z < minP.z) minP.z = p.z;
                    if(p.z > maxP.z) maxP.z = p.z;
                }
                
            }
        }
    }
    if(doDebugInfo) printf("Boundaries: (%f, %f, %f) - (%f, %f, %f)\n", minP.x, minP.y, minP.z, maxP.x, maxP.y, maxP.z);
}

//--------------------------------------------------------------
void setGradientMode(int g) {
    gradientMode = g;
    spaceTime.clear();


    switch(gradientMode) {
        case 1:
            gradientModeStr = "left-right";
            spaceNumCells.set(500, 1, 1);
            break;
            
        case 2:
            gradientModeStr = "right-left";
            spaceNumCells.set(500, 1, 1);
            break;
            
        case 3:
            gradientModeStr = "top-bottom";
            spaceNumCells.set(1, 500, 1);
            break;
            
        case 4:
            gradientModeStr = "bottom-top";
            spaceNumCells.set(1, 500, 1);
            break;
            
        case 5:
            gradientModeStr = "front-back";
            spaceNumCells.set(1, 1, 500);
            break;
            
        case 6:
            gradientModeStr = "back-front";
            spaceNumCells.set(1, 1, 500);
            break;
            
        case 7:
            gradientModeStr = "spherical";
            spaceNumCells.set(30, 30, 30);
            break;
            
        case 8:
            gradientModeStr = "random";
            spaceNumCells.set(40, 40, 40);
            break;
            
        case 9:
            gradientModeStr = "oldest";
            spaceNumCells.set(2);
            break;
            
        default:
            gradientModeStr = "most recent";
            spaceNumCells.set(2);
            break;
            
            
    }
    
    ofLog(OF_LOG_VERBOSE, "setGradientMode: " + ofToString(gradientMode) + " " + gradientModeStr + " (" + ofToString(spaceNumCells.x) + ", " + ofToString(spaceNumCells.y) + ", " + ofToString(spaceNumCells.z) + ")");
}


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetFrameRate(30);
    
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	usingKinect = kinect.init();
    if(usingKinect) {
        ofLog(OF_LOG_VERBOSE, "USING KINECT");
        usingKinect = kinect.open();		// opens first available kinect
        
        kinectAngle = kinect.getTargetCameraTiltAngle();
        grabber = &kinect;
        inputWidth = kinect.getWidth();
        inputHeight = kinect.getHeight();
    }
    
    if(usingKinect == false) {
        ofLog(OF_LOG_VERBOSE, "USING VIDEO GRABBER");
        videoGrabber.initGrabber(640, 480);
        grabber = &videoGrabber;
        inputWidth = videoGrabber.getWidth();
        inputHeight = videoGrabber.getHeight();
    }
    
    spaceTime.setMaxFrames(numScanFrames);
    setGradientMode(0);
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	
	grabber->update();
    
    if(doPause == false) {
        if(grabber->isFrameNew()) {

            if(doSlitScan) {
                // construct space time continuum
                msa::SpaceT<ofMesh> *space = new msa::SpaceT<ofMesh>(spaceNumCells, spaceBoundaryMin, spaceBoundaryMax);
                
                ofPixelsRef pixelsRef = grabber->getPixelsRef();
                // iterate all vertices of mesh, and add to relevant quantum cells
                for(int j=0; j<inputHeight; j += pixelStep) {
                    for(int i=0; i<inputWidth; i += pixelStep) {
                        ofVec3f p;
                        ofFloatColor c;
                        bool doIt;
                        if(usingKinect) {
                            p = kinect.getWorldCoordinateAt(i, j);
                            c = kinect.getColorAt(i, j);
                            doIt = kinect.getDistanceAt(i, j) > 0;
                        } else {
                            c = pixelsRef.getColor(i, j);
                            p.x = ofMap(i, 0, pixelsRef.getWidth(), spaceBoundaryMin.x, spaceBoundaryMax.x);
                            p.y = ofMap(j, 0, pixelsRef.getHeight(), spaceBoundaryMin.y, spaceBoundaryMax.y);
                            p.z = ofMap(c.getBrightness(), 1, 0, webcamNear, webcamFar);
                            doIt = true;
                        }
                        if(ofInRange(p.z, nearThreshold, farThreshold) && doIt) {
                            ofVec3f index = space->getIndexForPosition(p);
                            ofMesh &cellMesh = space->getDataAtIndex(index);
                            cellMesh.addVertex(p);
                            cellMesh.addColor(c);
                            if(doDebugInfo) {
                                int i = index.x;
                                int j = index.y;
                                int k = index.z;
                                if(cellMesh.getNumVertices()>0) printf("UPDATE SPACE pos: %f, %f, %f, cell: %i, %i, %i, numVertices: %i\n", p.x, p.y, p.z, i, j, k, cellMesh.getNumVertices());
                            }
                        }
                    }
                }
                
                // add space to space time continuum
                spaceTime.addSpace(space);
                
                // update mesh
                mesh.clear();
                for(int i=0; i<spaceNumCells.x; i++) {
                    for(int j=0; j<spaceNumCells.y; j++) {
                        for(int k=0; k<spaceNumCells.z; k++) {
                            float t;
                            switch(gradientMode) {
                                case 0:
                                    t = 0;  // use most recent mesh
                                    break;
                                    
                                case 1:
                                    t = i * 1.0f/spaceNumCells.x; // left to right
                                    break;
                                    
                                case 2:
                                    t = 1.0f - i * 1.0f/spaceNumCells.x; // right to left
                                    break;
                                    
                                case 3:
                                    t = j * 1.0f/spaceNumCells.y; // up to down
                                    break;
                                    
                                case 4:
                                    t = 1.0f - j * 1.0f/spaceNumCells.y; // down to up
                                    break;
                                    
                                case 5:
                                    t = k * 1.0f/spaceNumCells.z; // front to back
                                    break;
                                    
                                case 6:
                                    t = 1.0f - k * 1.0f/spaceNumCells.z; //  back to front
                                    break;
                                    
                                case 7:
                                {
                                    // spherical
                                    float tx = i/spaceNumCells.x * 2 - 1;
                                    float ty = j/spaceNumCells.y * 2 - 1;
                                    float tz = k/spaceNumCells.z * 2 - 1;
                                    t = tx * tx + ty * ty + tz * tz;
//                                    t = sqrt(t);
                                }
                                    break;
                                    
                                case 8:
                                    t = ofRandomuf();
                                    break;
                                    
                                case 9:
                                    t = 1;
                                    break;
                                    
                            }
                            
                            t = ofClamp(t, 0, 1);
                            
                            ofMesh &cellMesh = spaceTime.getSpaceAtTime(t)->getDataAtIndex(i, j, k);
                            
                            mesh.addVertices(cellMesh.getVertices());
                            mesh.addColors(cellMesh.getColors());
                            
                            if(doDebugInfo) {
                                if(cellMesh.getNumVertices()>0) printf("UPDATE MESH cell: %i, %i, %i, time: %f, numVertices: %i\n", i, j, k, t, cellMesh.getNumVertices());
                            }
                        } // k
                    } // j
                } // i
                
            } else {
                mesh.clear();
                if(usingKinect) fillMeshFromKinect(mesh, kinect);
                else fillMeshFromPixels(mesh, grabber->getPixelsRef());
            }
        }
    }
    
    if(doSaveMesh) {
        doSaveMesh = false;
        mesh.save("mesh_" + ofGetTimestampString() + ".ply");
    }
}

//--------------------------------------------------------------
void testApp::draw() {
    
    ofSetColor(255, 255, 255);

    if(usingKinect) kinect.draw(ofGetWidth()-160, 0, 160, 120);
    else videoGrabber.draw(ofGetWidth()-160, 0, 160, 120);
    
    if(doDrawPointCloud) {
        easyCam.begin();
        
        glPointSize(3);
        ofPushMatrix();
        // the projected points are 'upside down' and 'backwards'
        ofScale(1, -1, -1);
        float s = 0.8;
        ofScale(s, s, s);
        ofTranslate(0, 0, -1000); // center the points a bit
        glEnable(GL_DEPTH_TEST);
        
        mesh.drawVertices();

        glDisable(GL_DEPTH_TEST);
        ofPopMatrix();
        
        easyCam.end();
    }
    
    // draw instructions
    ofSetColor(255, 255, 255);
    stringstream reportStream;
    reportStream << ""
    << "nearThreshold (,.)    : " << nearThreshold << endl
    << "farThreshold (<>)     : " << farThreshold << endl
    << "fps                   : " << ofGetFrameRate() << endl
    << "kinectAngle (UP/DOWN) : " << kinectAngle << endl
    << "doPause (p)           : " << doPause << endl
    << "doSlitScan (s)        : " << doSlitScan << endl
    << "doDrawPointCloud (c)  : " << doDrawPointCloud << endl
    << "doDebugInfo (d)       : " << doDebugInfo << endl
    << endl
    << "gradientMode (0-9)    : " << gradientMode << ": " << gradientModeStr << endl
    << "   0: most recent" << (gradientMode == 0 ? " * " : "" ) << endl
    << "   1: left-right" << (gradientMode == 1 ? " * " : "" ) << endl
    << "   2: right-left" << (gradientMode == 2 ? " * " : "" ) << endl
    << "   3: top-bottom" << (gradientMode == 3 ? " * " : "" ) << endl
    << "   4: bottom-top" << (gradientMode == 4 ? " * " : "" ) << endl
    << "   5: front-back" << (gradientMode == 5 ? " * " : "" ) << endl
    << "   6: back-front" << (gradientMode == 6 ? " * " : "" ) << endl
    << "   7: spherical" << (gradientMode == 7 ? " * " : "" ) << endl
    << "   8: random" << (gradientMode == 8 ? " * " : "" ) << endl
    << "   9: oldest" << (gradientMode == 9 ? " * " : "" ) << endl
    << endl;
    
    ofDrawBitmapString(reportStream.str(), 20, 20);
}

//--------------------------------------------------------------
void testApp::exit() {
    kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
    switch (key) {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            setGradientMode(key-'0');
            break;
            
        case'p':
            doPause = !doPause;
            break;
            
        case '>':
            farThreshold += 10;
			if (farThreshold > 10000) farThreshold = 10000;
            printf("farThreshold: %f\n", farThreshold);
            break;
            
        case '<':
            farThreshold -= 10;
			if (farThreshold < 0) farThreshold = 0;
            printf("farThreshold: %f\n", farThreshold);
            break;
            
        case '.':
            nearThreshold += 10;
			if (nearThreshold > 10000) nearThreshold = 10000;
            printf("nearThreshold: %f\n", farThreshold);
            break;
            
        case ',':
            nearThreshold -= 10;
			if (nearThreshold < 0) nearThreshold = 0;
            printf("nearThreshold: %f\n", farThreshold);
            break;
            
        case 'S':
            doSaveMesh = true;
            break;
            
        case 's':
            doSlitScan ^= true;
            break;
            
        case 'c':
            doDrawPointCloud ^= true;
            break;
            
        case 'd':
            doDebugInfo ^= true;
            break;
            
        case OF_KEY_UP:
            kinectAngle++;
            if(kinectAngle>30) kinectAngle=30;
            kinect.setCameraTiltAngle(kinectAngle);
            break;
            
        case OF_KEY_DOWN:
            kinectAngle--;
            if(kinectAngle<-30) kinectAngle=-30;
            kinect.setCameraTiltAngle(kinectAngle);
            break;
    }
}
