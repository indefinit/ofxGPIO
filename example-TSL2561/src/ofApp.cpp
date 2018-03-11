#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	lightSensor = make_shared<TSL2561>(TSL2561_ADDR_FLOAT, 12345);
	if(lightSensor->begin()){
		lightSensor->enableAutoRange(true);
		lightSensor->setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
	}
	
}

//--------------------------------------------------------------
void ofApp::update(){
	/* Get a new sensor event */
	sensors_event_t event;
	lightSensor->getEvent(&event);
	/* Display the results (light is measured in lux) */
	if (event.light)
	{
		ofLog() << event.light << " lux" << endl;
	}
	else
	{
		/* If event.light = 0 lux the sensor is probably saturated
		 and no reliable data could be generated! */
		ofLog() << "Sensor overload" << endl;
	}
}

//--------------------------------------------------------------
void ofApp::draw(){

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
