#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	pBus = std::make_shared<I2c>("/dev/i2c-1");
	pBus->addressSet(0x39);
	// Power ON mode(0x03)
	char config[2] = {0};
	config[0] = 0x00 | 0x80;
	config[1] = 0x03;
	pBus->writeByte((uint8_t) config[0],(uint8_t)config[1]);
	
	// Select timing register(0x01 | 0x80)
	// Nominal integration time = 402ms(0x02)
	config[0] = 0x01 | 0x80;
	config[1] = 0x02;
	pBus->writeByte((uint8_t) config[0], config[1]);
	usleep(100);
}

//--------------------------------------------------------------
void ofApp::update(){
	// Read 4 bytes of data from register(0x0C | 0x80)
	// ch0 lsb, ch0 msb, ch1 lsb, ch1 msb
	uint8_t reg[1] = {0x0C | 0x80};
	uint8_t data[4] = {0};
	//	pBus->write(reg);
	pBus->readBlock(reg, 4,data);
	// Convert the data
	float ch0 = (data[1] * 256 + data[0]);
	float ch1 = (data[3] * 256 + data[2]);
	// Output data to screen
	ofLog() <<"Full Spectrum(IR+Visible) : " << ch0 << " lux" << endl;
	ofLog() << "Infrared Value : " << ch1 << " lux" << endl;
	ofLog() << "Visible Value : " << ch0-ch1 << " lux" << endl;
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
