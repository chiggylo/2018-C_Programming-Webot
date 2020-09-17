#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/differential_wheels.h>
#include <stdbool.h>
#include <webots/gps.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <webots/keyboard.h>
#include <math.h>
#include <string.h>
#include <webots/camera.h>
#define TIME_STEP 64
#define SPEED 10 // speed of robot movement
#define MAX 100 // array size

// dimension of robot setting
#define robotSize 0.05
#define sensorLength = 0.1;
const double error = robotSize/10;

// structure for coordinate
struct  Coordinate{
	double x;
	double z;
	char *direction;
};

// array of structure for recording new pathways
struct Coordinate newPath[MAX];
int top = -1;
void push();
void pop();
void display();

// -----adding coordinates------------ //
	void push(double x, double z, char* direction){
	if(top == MAX-1){
		printf("Stack is full\n");
	}
	else{
		top = top + 1;
		newPath[top].x = x;
		newPath[top].z = z;
		newPath[top].direction = direction;
	}
	printf("Number of new path record: %d\n", top+1);
}

// ----- deleting coordinates ------- //
void pop(){
	if(top == -1){
		printf("Stack is empty");
	}
	else{
		top = top - 1;
	}
	printf("Deleting new path...\nNumber of new path record: %d\n", top+1);
}

// array for backtracking
struct Coordinate trackPath[MAX];
void pushTrack();
void popTrack();
int topTrack = -1;

// -----adding coordinates------------ //
void pushTrack(double x, double z, char* direction){
	if(topTrack == MAX-1){
		printf("Stack is full");
	}
	else{
		topTrack = topTrack + 1;
		trackPath[topTrack].x = x;
		trackPath[topTrack].z = z;
		trackPath[topTrack].direction = direction;
	}
	printf("Number of track path record: %d\n", topTrack+1);
}

// ----- deleting coordinates ------- //
void popTrack(){
	if(topTrack == -1){
		printf("Stack is empty\n");
	}
	else{
		topTrack = topTrack - 1;
	}
	printf("Deleting track path...\nNumber of track path record: %d\n", topTrack+1);
}
enum BLOB_TYPE {RED,GREEN,BLUE,NONE}; // red = danger, green = survivor, blue = door

typedef enum{ GPS, SUPERVISED } gps_mode_types;
typedef enum {AUTOPILOT, MANUAL} Mode;

int main(int argc, char **argv){
	wb_robot_init();
	
	// GPS
	WbDeviceTag gps = wb_robot_get_device("gps");
	wb_gps_enable(gps, TIME_STEP);
	
	// Receiver
	WbDeviceTag receiver = wb_robot_get_device("receiver");
	wb_receiver_enable(receiver, TIME_STEP);
	
	// Sensor
	WbDeviceTag  front = wb_robot_get_device("front");
	wb_distance_sensor_enable(front, TIME_STEP);
	WbDeviceTag  back = wb_robot_get_device("back");
	wb_distance_sensor_enable(back, TIME_STEP);
	WbDeviceTag  right = wb_robot_get_device("right");
	wb_distance_sensor_enable(right, TIME_STEP);
	WbDeviceTag  left = wb_robot_get_device("left");
	wb_distance_sensor_enable(left, TIME_STEP);
	
	// Keyboard
	wb_keyboard_enable(TIME_STEP); 
	
	// camera initialisation
	WbDeviceTag camera = wb_robot_get_device("camera(1)");
	wb_camera_enable(camera, 2*TIME_STEP);
	
	// robot triggers
	bool backward = false; // false = forward, true = backward (start with forward)
	bool scanning = true;
	bool event = false;
	bool backtracking = false;
	bool survivor = false;
	
	// new Pathway at current location (top priority)
	bool wallTurnLeft = false;
	bool wallTurnRight = false;
	bool wall = false;
	
	// new Pathway at different location (low priority
	bool opening = false;
	bool rOpening = false;
	bool lOpening = false;
	double gap1[2]= {0.0,0.0};
	
	// turning counter: SPEED = 10 -> counter <30
	int counter = 0;
	
	// camera setting
	int width, height;
	int red,blue,green;
	const unsigned char *image;
	enum BLOB_TYPE current_blob;
	
	// assign camera setting to variable to be used
	width = wb_camera_get_width(camera);
	height = wb_camera_get_height(camera);
	
	// set robot mode to autopilot when first start
	Mode mode = AUTOPILOT;
	
	while (wb_robot_step(TIME_STEP) != -1) {	
		// movement speed initialisation
		double left_speed, right_speed; // -SPEED = forward, SPEED = backward
		
		// current location of robot initialisation
		const double *currentLoc = wb_gps_get_values(gps); // current location: [0] = xLoc, [1] = yLoc, [2] = zLoc
		
		// keyboard use
		int c;
		do {
			c = wb_keyboard_get_key();
			if (c == -1){
				break;
			}
			mode = MANUAL;
			survivor = false;
			switch (c) {
				case 'A':
					mode = AUTOPILOT; // set autopilot
					break;
				case WB_KEYBOARD_UP: // move forward
					left_speed = -SPEED;
					right_speed = -SPEED;
					break;
				case WB_KEYBOARD_DOWN: // move backward
					left_speed = SPEED;
					right_speed = SPEED;
					break;
				case WB_KEYBOARD_RIGHT: // turn right
					left_speed = SPEED;
					right_speed = -SPEED;
					break;
				case WB_KEYBOARD_LEFT: // turn left
					left_speed = -SPEED;
					right_speed = SPEED;
					break;
				case 'S': // stop movement
					left_speed = 0;
					right_speed = 0;
					break;
			}
		} while (c);
		
		// resetting variable (camera)
		red = 0;
		green = 0;
		blue = 0;
		
		// record image from camera (front)
		image = wb_camera_get_image(camera);
		
		// object identification - danger, door and survivor
		for(int i=width/3; i<2*width/3; i++){ 
			for(int j=height/2; j<3*height/4; j++){
				red += wb_camera_image_get_red(image, width,i,j);
				blue += wb_camera_image_get_blue(image, width,i,j);
				green += wb_camera_image_get_green(image, width,i,j);
			}
		}
		
		if((red>3*green) && (red > 3*blue)){ // find if there's anything needed to check, danger/survivor/door
			current_blob = RED;
		}
		else if((green>3*red)&&(green>3*blue)){
			current_blob=GREEN;
		}
		else if((blue>3*red)&&(blue>3*green)){
			current_blob=BLUE;
		}
		else{
			current_blob = NONE;
		}
		
		// sensor feedback value (1000 = nothing, 565 <= closest distance)
		double front_value = wb_distance_sensor_get_value(front);
		double back_value = wb_distance_sensor_get_value(back);
		double right_value = wb_distance_sensor_get_value(right);
		double left_value = wb_distance_sensor_get_value(left);
		if(mode == AUTOPILOT){
			// starting movemment
			if(!backward){ // always move forward while scanning (true)
				left_speed = -SPEED;
				right_speed= -SPEED;
				
				//events
				// move through door
				if(current_blob == BLUE){ // door detection
					if(!event){ // announce once, downside - can't do multiple event continuously
						printf("Door Detected! Moving through door...\n");
					}
					left_speed = -SPEED;
					right_speed = -SPEED;
					event = true;
				}
				else if(current_blob == RED){ // danger - fire in this scenario as it's the easily to detect in real life
					if(!event){	
						printf("DANGER AHEAD AT %.3f %.3f! DANGER AHEAD AT %.3f %.3f!\n", currentLoc[0], currentLoc[2], currentLoc[0], currentLoc[2]);
					}
					if(!backtracking){
						char* direction = "backward";
						pushTrack(currentLoc[0],currentLoc[2], direction);
					}
					backtracking = true;
					backward = true;
					scanning = false;
					
					// stop movement - will be overidden
					left_speed = 0;
					right_speed = 0;
				}
				else if(current_blob == GREEN){
					if(!event){
						printf("SURVIVOR FOUND AT %.3f %.3f! SURVIVOR FOUND AT %.3f %.3f!\nManual Control Activated\nControl...\nUp: Forward\nDown: Backward\nRight: Turn Right\nLeft: Turn Left\nS: Stop\nA:Autopilot", currentLoc[0], currentLoc[2], currentLoc[0], currentLoc[2]);
					}
					survivor = true;
					event = true;
				}
				else{ // no event detected
					event = false;
				}	
				if(scanning){ // scanning for newpath
					if(right_value == 1000 && !rOpening){ // beginning of openning
						rOpening = true;
						// put into x and y coordinate of the start opening
						gap1[0]=currentLoc[0]; 
						gap1[1]=currentLoc[2];
					}
					else if(right_value != 1000 && rOpening){ // end of opening
						rOpening = false;
						double tempX = (gap1[0]+currentLoc[0])/2;
						double tempZ = (gap1[1]+currentLoc[2])/2;
						char* direction = "right";
						double checkX = sqrt(pow(gap1[0] - currentLoc[0],2));
						double checkZ = sqrt(pow(gap1[1] - currentLoc[2],2));
						if(checkX > robotSize + error || checkZ > robotSize + error){ 
							push(tempX, tempZ, direction);
							printf("New pathway at the right recorded at %.3f %.3f\n", tempX, tempZ);
						}
					}
					if(left_value == 1000 && !lOpening){ // beginning of openning
						lOpening = true;
						// put into x and y coordinate of the start opening
						gap1[0]=currentLoc[0]; 
						gap1[1]=currentLoc[2];
					}
					else if(left_value != 1000 && lOpening){ // end of opening
						lOpening = false;
						double tempX = (gap1[0]+currentLoc[0])/2;
						double tempZ = (gap1[1]+currentLoc[2])/2;
						char* direction = "left";
						double checkX = sqrt(pow(gap1[0] - currentLoc[0],2));
						double checkZ = sqrt(pow(gap1[1] - currentLoc[2],2));
						if(checkX > robotSize + error || checkZ > robotSize + error){ 
							push(tempX, tempZ, direction);
							printf("New pathway at the left recorded at %.3f %.3f\n", tempX, tempZ);
						}
					}
				}
				else{
					if(wall){
						if(sqrt(pow(trackPath[topTrack].x - currentLoc[0],2)) > 0.13){ // size of robot/2 + distance sensor range + 0.05 error
							scanning = true;
							printf("Start scanning\n");
							opening = false;
						}
						if(sqrt(pow(trackPath[topTrack].z - currentLoc[2],2)) > 0.13){ // size of robot/2 + distance sensor range + 0.05 error
							scanning = true;
							printf("Start scanning\n");
							opening = false;
						}
						else{
							left_speed = -SPEED;
							right_speed = -SPEED;
						}
					}
					else{
						if(opening){ // if not allowed to scan = moving forward after turning into a newLoc , turning is also hitting this
							if(sqrt(pow(newPath[top].x - currentLoc[0],2)) > 0.13){ // size of robot/2 + distance sensor range + 0.05 error
								scanning = true;
								printf("Start scanning\n");
								pop();
								opening = false;
							}
							else if(sqrt(pow(newPath[top].z - currentLoc[2],2)) > 0.13){ // size of robot/2 + distance sensor range + 0.05 error
								scanning = true;
								printf("Start scanning\n");
								pop();
								opening = false;
							}
							else{
								left_speed = -SPEED;
								right_speed = -SPEED;
							}
						}
					}
				}
				if(front_value < 565){ // move as close to the wall possible
					printf("Starting to move backward and stop scanning\n");
					scanning = false; // stop scanning for pathway
					backward = true; // start moving backward
					
					//recording
					char* direction = "backward";
					pushTrack(currentLoc[0],currentLoc[2], direction);
					printf("Recording moving backward location for backtracking\n");
					if(left_value == 1000){ // check for newPath 
						wallTurnLeft = true;
						rOpening = false; // stop recording and move into it
						lOpening = false;
						opening = false;
						direction = "right";
						printf("Recording turning right location for backtracking\n");
						pushTrack(currentLoc[0],currentLoc[2], direction);
					}
					if(right_value == 1000){
						wallTurnRight = true;
						rOpening = false; // stop recording
						lOpening = false;
						opening = false;
						direction = "left";
						printf("Recording turning left location for backtracking\n");
						pushTrack(currentLoc[0],currentLoc[2], direction);
					}
				}			
			}
			else{ // moving backward and for turning
				left_speed = SPEED;
				right_speed = SPEED;
				if(backtracking){
					if(top != -1){
						for(int i = top; i >= 0 ; --i){
							if(sqrt(pow(newPath[i].x - currentLoc[0],2)) < 0.01 && sqrt(pow(newPath[i].z - currentLoc[2],2)) < 0.01){
								printf("New path selected!! Resume Searching\n");
								backtracking = false;
							}
						}
					}
					if(strcmp(trackPath[topTrack].direction, "backward")==0){
						if(sqrt(pow(trackPath[topTrack-1].x - currentLoc[0],2)) > 0.015){ 
							left_speed = SPEED;
							right_speed = SPEED;
						}
						else if(sqrt(pow(trackPath[topTrack-1].z - currentLoc[2],2)) > 0.015){ 
							left_speed = SPEED;
							right_speed = SPEED;
						}
						else{
							popTrack();
						}
					}
					else if(strcmp(trackPath[topTrack].direction, "right")==0){
						if(counter<30){ // turn 90 degree, 10 SPEED = 30 Turns
							left_speed = -SPEED;
							right_speed = SPEED;
						}
						counter = counter +1;
					}
					else if(strcmp(trackPath[topTrack].direction,"left")==0){
						if(counter<30){ // turn 90 degree, 10 SPEED = 30 Turns
							left_speed = -SPEED;
							right_speed = SPEED;
						}
						counter = counter +1;
					}
					if(counter == 30){
						counter = 0;
						popTrack();
					}
					if(topTrack == -1){
						left_speed = 0;
						right_speed = 0;
					}
				}
				else{
					if(wallTurnLeft || wallTurnRight){
						if(wallTurnLeft && !scanning){
							if(counter<30){ // turn 90 degree, 10 SPEED = 30 Turns
								left_speed = -SPEED;
								right_speed = SPEED;
							}
							counter = counter +1;
						}
						else if(wallTurnRight && !scanning){
							if(counter<30){ // turn 90 degree, 10 SPEED = 30 Turns
								left_speed = SPEED;
								right_speed = -SPEED;
							}
							counter = counter +1;
						}
						if(counter == 30){ 
							counter = 0;
							printf("Finished wall turning!\n");
							wallTurnLeft = false;
							wallTurnRight = false;
							backward = false;
							scanning = false; // stop scanning until move beyond the sensor point
							wall = true;
						}
					}
					else{
							if(top != -1){ // if newpath, go to it (lowest priority)
								double check = sqrt(pow(currentLoc[0]-newPath[top].x,2));
								double check2 = sqrt(pow(currentLoc[2]-newPath[top].z,2));
								if(check > 0.015){ // move toward the midpoint at a 0.015 diff, due to inaccuracy of turning, the difference needed to be larger to compensate
									left_speed = SPEED;
									right_speed = SPEED;
								}
								else if(check2 > 0.015){
									left_speed = SPEED;
									right_speed = SPEED;
								}
								else{
									if(counter<30){ // turn 90 degree, 10 SPEED ~ 30 Turns (not exactly which meant inaccuracy builds up over distance or repeat turn
										counter = counter +1;
										if(strcmp(newPath[top].direction, "right") == 0){
											left_speed = SPEED;
											right_speed = -SPEED;
										}
										else if(strcmp(newPath[top].direction, "left") == 0){
											left_speed = SPEED;
											right_speed = -SPEED;
										}
									}
								}
								if(counter == 30){ // finish turning
									if(strcmp(newPath[top].direction, "right") == 0){
										char* direction = "left";
										pushTrack(currentLoc[0], currentLoc[2], direction);
									}
									else if(strcmp(newPath[top].direction, "left") == 0){
										char* direction = "right";
										pushTrack(currentLoc[0], currentLoc[2], direction);
									}
									printf("Finished open turning!\n");
									backward = false;
									scanning = false; // stop scanning until move beyond the sensor point
									counter = 0;
									opening = true;
								}
							}
						else{
							if(back_value < 565){ // move forward when backed against a wall
								backward = false;
							}
						}
					}
				}
			}
		}
		if(survivor){
			left_speed = 0;
			right_speed = 0;
		}
		wb_differential_wheels_set_speed(left_speed, right_speed);
	};
	wb_robot_cleanup();
	return 0;
}
