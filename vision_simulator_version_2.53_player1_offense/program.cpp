////
//FINAL VERSION <-- program (cpp) [Offense - Player 1]
//TEAM DeathStar (MECH 6631 - Robot Light Wars)
//		||Members
//		||Chandan Satija
//		||Tasnim Haque
//		||Farshad Zaboli
//		||Mohammad Saeidi


#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>


using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer5.h"

#include "DeathStar.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

extern robot_system S1;

int process_image(image &a, image &b, image &rgb, image &rgb0, image &rgb_a, image &rgb_b, image &label, double &tc, double &tc0, double *hist, int &nhist, double &hmin, double &hmax, int &nlabels);
int set_images(image &a, image &b, image &rgb, image &rgb0, image &rgb_a, image &rgb_b, image &label, image &mask, image &rgb_mask, image &edge_mag,image &edge_theta,int width, int height, int mask_width, int mask_height);
int allocate_images(image &a, image &b, image &rgb, image &rgb0, image &rgb_a, image &rgb_b, image &label, image &mask, image &rgb_mask, image &edge_mag, image &edge_theta);
int free_images(image &a, image &b, image &rgb, image &rgb0, image &rgb_a, image &rgb_b, image &label, image &mask, image &rgb_mask, image &edge_mag, image &edge_theta);

//Global Variables
//Robot Position
double init_x0 = 450;
double init_y0 = 350;
double init_theta0 = 3.14159 / 6;

// No of Obstacles
int Ob_Number = 1;

// Obstacle 1 Position
double init_x_obs1 = 150;
double init_y_obs1 = 300;

// Obstacle 2 Position
//double init_x_obs2 = 430;
//double init_y_obs2 = 110;

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc,tc0; // clock time

	/// FOR MULTIPLAYER PLAYER MODE
	int mode, level;
	
	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1  = 640;
	height1 = 480;
	
	// number of obstacles
	N_obs = Ob_Number;

	x_obs[1] = init_x_obs1; // pixels
	y_obs[1] = init_y_obs1; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	//x_obs[2] = init_x_obs2; // pixels
	//y_obs[2] = init_y_obs2; // pixels
	//size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////
	D = 121.0; // distance between front wheels (pixels)
	
	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;
	
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	
	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	activate_simulation(width1,height1,x_obs,y_obs,size_obs,N_obs,
		"robot_A.bmp","robot_B.bmp","background.bmp","obstacle.bmp",D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);	

	// open an output file if needed for testing or plotting

	///// FOR PLAYER MODE
	//	ofstream fout("sim1.txt");
	//	fout << scientific;
	mode = 1;
	level = 1;
	set_simulation_mode(mode, level);
	
	// set robot initial position (pixels) and angle (rad)
	x0 = init_x0;
	y0 = init_y0;
	theta0 = init_theta0;
	set_robot_position(x0,y0,theta0);

	///// FOR PLAYER MODE
	// set opponent initial position (pixels) and angle (rad)
    //	x0 = 150;
    //	y0 = 375;
    //	theta0 = 3.14159/4;
    //	set_opponent_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)
	
	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l,pw_r,pw_laser,laser,
		light,light_gradient,light_dir,image_noise,
		max_speed,opponent_max_speed);

	cout << "\npress space key to begin program.";
	pause();
	// For PLAYER MODE
	wait_for_player();
	///////////////////////////////// regular vision program ////////////////////////////////

	int c1[6 + 1], c2[6 + 1], c3[6 + 1], c4[6 + 1];
	initialize_c_array(c1, c2, c3, c4);
	
	// Image Structure Decleration
	
	image a, b, rgb, rgb0, rgb_a, rgb_b; // declare some image structures
	image label;
	ibyte *pb;
	ibyte *prgb0;
	image mask, rgb_mask, edge_mag, edge_theta;
	ibyte *pmask, *pedge, *prgb_mask;

	int nhist, j, nlabels;
	double hist[255], hmin, hmax, x, ic, jc;
	int height, width;
	int R, G, B;

	int th, k;

	// Mask variable
	int imask = 0, jmask = 0;
	int mask_ci, mask_cj, mask_width , mask_height ;
	double mask_angle = 0;
	int i_l, j_l;
	int ig, jg;
	int mask_pixel;
	int mi_inside, mj_inside;
	int mi_target, mj_target;
	int attach_mask_x, attach_mask_y;
	double stopping_distance_obstacle,stopping_distance_opp;

	//Set Mask Dimensions and Stopping Distance according to obstacles size
	mask_width = 200;
	mask_height = 160;
	stopping_distance_obstacle = 160.00;
	stopping_distance_opp = 80.00;

	// Control Variables
	int x_robot = 0, y_robot = 0;
	double phi_robot = 0, phi_opp = 0;
	double xgoal, ygoal;
	double phi_goal_local;
	double e_orientation = 0;
	double e_orientation_local_goal = 0;
	double goal_distance;
	bool flipper,juggler = 0;
	int x_obs_near, y_obs_near, num_near;

	// PID Control Variables
	double tp,dt;
	static double ep = 0, ei = 0;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width  = 640;
	height = 480;

	set_images(a,b,rgb,rgb0,rgb_a,rgb_b,label,mask,rgb_mask,edge_mag,edge_theta,width,height,mask_width,mask_height);

	allocate_images(a, b, rgb, rgb0, rgb_a, rgb_b, label, mask, rgb_mask, edge_mag, edge_theta);

	// measure initial clock time
	tc0 = high_resolution_time(); 

	// Process Image once for initialization before entering into the loop
	process_image(a, b, rgb, rgb0, rgb_a, rgb_b, label, tc, tc0, hist, nhist, hmin, hmax, nlabels);
		
	// For obstacle size function
	int x_size1[10 + 1];
	int x_size2[10 + 1];
	int y_size1[10 + 1];
	int y_size2[10 + 1];

	// Initliaze with bad values 
	int x_obs_array[10 + 1] ;
	int y_obs_array[10 + 1] ;

	//centroidTracking 
	cout << "\nTeam Deathstar";

	//centroid_tracking()			
	int robot_a[2 * 4 + 1], robot_b[2 * 4 + 1], obstacles[2 * 4 + 1], obstacles_size[2 + 1];
	int size_r = 2;
	int size_o = nlabels - (4 * 2);
	initialize_robot_arrays(robot_a, robot_b, size_r);
	initialize_obstacle_arrays(obstacles, obstacles_size, size_o);
			
	//filter which objects to track;
	filter_objects(nlabels, label, x_size1, x_size2, y_size1, y_size2, b, rgb0, c1, c2, c3, c4, robot_a, robot_b, obstacles, obstacles_size, size_r, size_o);

	int nlabels_actual = nlabels;	//was intended for the fitler recovery
	int d_SF = 5;			//distance safety factor (for defend)


	while (1)
		{
				//Track own robot 
				track_object(robot_a, size_r, label, b, rgb0);
				int xgreen = robot_a[3];	
				int ygreen = robot_a[4];	
				int xred = robot_a[7];		
				int yred = robot_a[8];	

				//Track Opponent
				track_object(robot_b, size_r, label, b, rgb0);
				int xorange = robot_b[3];	
				int yorange = robot_b[4];	
				int xblue = robot_b[7];		
				int yblue = robot_b[8];	
				
				int x_opp = xorange;
				int y_opp = yorange;

				//track obstacles
				track_object(obstacles, size_o, label, b, rgb0);
				 
				//Obstacle Number
				int obs_number = size_o;

				//Obstacle 1
				x_obs_array[1] = obstacles[3];
				y_obs_array[1] = obstacles[4];
				//Obstacle 2
				x_obs_array[2] = obstacles[7];
				y_obs_array[2] = obstacles[8];

				// Note :- All angle are in degree and belong to range (-180 to 180] degrees
				// phi_robot =Taken from Red to Green Centroid
				phi_robot = atan2(ygreen - yred, xgreen - xred)*(180.00 / 3.14159);
				// phi_opponent =Taken from Blue to Orange Centroid
				phi_opp = atan2(yorange - yblue, xorange - xblue)*(180.00 / 3.14159);

				// Co-ordinates of robot axis of rotation of  halfway between wheels
				// x_robot,y_robot  in workspace frame.
				// If Ax = 0 , x_robot = x_green y_robot = ygreen
	
				x_robot = xgreen + cos((phi_robot / 180.00)*3.14159)*Ax;
				y_robot = ygreen + sin((phi_robot / 180.00)*3.14159)*Ax;

				// Attach mask to a reference point
				// NOTE: x_robot,y_robot is the reference point to which a mask is attached
				//		 mask_height and mask_width can changed while declaring the variables
				attach_mask_x = x_robot;
				attach_mask_y = y_robot;

				// Gives mask frame origin coodinates and mask angle wrt to a reference point
				set_mask(attach_mask_x, attach_mask_y, mask_height, phi_robot, mask_ci, mask_cj, mask_angle);
				
				// Create Mask such that it always infront of the robot along the heading direction
				// The is like having a virtual laser scanner in front of the robot using vision feedback
				create_mask(mask_ci, mask_cj, mask_height, mask_width, mask, b, mask_angle);

				// Pixel in masks can accessed and processed seperately e.g sobel edge 
				//sobel(mask, edge_mag, edge_theta);

				//Mask Visualizer- Good for debugging and visualtisation for control logic inside of the mask
				//mask_visualizer(edge_mag, rgb0, mask_ci, mask_cj,mask_angle);

				//***********************************************************
				//    USE OF MASK - Control Logic 
				//*****************************************************************

				// Mask can used to detect a nearby obstacles that is in front of robot
				// and approriate response can be genereted (Local Planning)

				xgoal = xorange;
	
				ygoal = yorange;

				// x_robot //y_robot - position drive axis midpoint used in kinematic modelling modelling
				double phi_goal = atan2(ygoal - y_robot, xgoal - x_robot)*(180.00 / 3.14159);

				e_orientation = phi_goal - phi_robot;
				
				goal_distance = sqrt((double)(x_robot - xgoal)*(x_robot - xgoal) + (y_robot - ygoal)*(y_robot - ygoal));
							
				// Find Nearest obstacle that may happen to be in the mask
				double obs_distance[2 + 1];
				
				obs_distance[1] = sqrt((double)(x_robot - x_obs_array[1])*(x_robot - x_obs_array[1]) + (y_robot - y_obs_array[1])*(y_robot - y_obs_array[1]));
				obs_distance[2] = sqrt((double)(x_robot - x_obs_array[2])*(x_robot - x_obs_array[2]) + (y_robot - y_obs_array[2])*(y_robot - y_obs_array[2]));
				
				// Check for lowest distance value with below switch
				if (obs_distance[1] > obs_distance[2]) num_near = 2;

				else if (obs_distance[2] >= obs_distance[1]) num_near = 1;

				else num_near = 0;

				// Check if the nearest obstacle is within the mask or not 
				// Case1) If Yes, Execute obstacle Response strategy
			    // Case2)-If No, Proceed with Global planning 
				

				//How to check if obstacle or target is within the mask
				// Step 1) Call below function 
				tf_workspace_to_mask(x_obs_array[num_near], y_obs_array[num_near], mask_ci, mask_cj, mi_inside, mj_inside, mask_angle);
				// x_obs,y_obs - Obstacles co-ordinates in world frame
				// mi_inside,mj_inside - Obstacles co-ordinates in mask frame
				// Control Logic will taken locally w.r.t mask frame
				// For target call below (Just supply the x,y cordinates of object of interest to the function)
				tf_workspace_to_mask(xorange, yorange, mask_ci, mask_cj, mi_target, mj_target, mask_angle);
				// xorange,yorange - Target co-ordinates in world frame
				// mi_target,mj_target - Obstacles co-ordinates in mask frame
				// Control Logic will taken locally w.r.t mask frame
				
				//Step 2) If obstacle/target is inside the mask , below if statement will be true.

				// For Obstacle inside the mask 
				if ((mi_inside > 0 && mi_inside < mask_width) && (mj_inside >0 && mj_inside < mask_height))
						{
							
							if (obs_distance[num_near]< stopping_distance_obstacle) //size_ob + buffer to avoid bumping into the object
							{
								stop_robot(pw_l, pw_r);
							}

							//ALIGN_LOCK
							// This is a locking function that uses PID Control to keep it stable
							// Below function will lock to goal angle with -+10 degrees precision
							// NOTE : Call set dt everytime before calling this function
							dt = high_resolution_time() - tc;
							align_lock(phi_robot, phi_goal, dt, pw_l, pw_r, ep, ei, 10.0);

						}

				//For Targer inside the mask
				else if ((mi_target > 0 && mi_target < mask_width) && (mj_target>0 && mj_target < mask_height))
				{
						if (goal_distance < stopping_distance_opp) //size_target + buffer to avoid bumping into the opponent
						{
							stop_robot(pw_l, pw_r);
						}

						// Align to toward the opponent
						align_fast(phi_robot, phi_goal, pw_l, pw_r, 3.0);

						// This ton ensure that the we hit the target and since the target is already inside the mask
						if (e_orientation <= 5.00 && e_orientation >= -5.00)
						{
							laser = 1;
						}
				}

				// GLOBAL PLANNING LOGIC -
				// To control the movement of the function irrespective of the fact that something is within mask or not
				else
						{
							//START GLOBAL PLAN LOGIC
							// Align Robot towards goal and march forward

							double error_tolerance = 3.0;

							e_orientation = phi_goal - phi_robot;

							//Align if out of Tolerance 
							if (e_orientation >= error_tolerance || e_orientation <= -error_tolerance)
							{
								align_full_throttle(phi_robot, phi_goal, pw_l, pw_r);
							}

							else// Move forward 
							{
								move_forward(pw_l, pw_r, ep, ei);
							}

							
							if (goal_distance < stopping_distance_opp)//size_target + buffer to avoid bumping into the opponent
							{
								stop_robot(pw_l, pw_r);
							}

						}


				// Limit Robot from going from the screen
				if (xred < 0 + 5) move_forward(pw_l, pw_r, ep, ei);
				if (xred > width - 1 - 5)move_forward(pw_l, pw_r, ep, ei);
				if (yred < 0 + 5)move_forward(pw_l, pw_r, ep, ei);
				if (yred > height - 1 - 5)move_forward(pw_l, pw_r, ep, ei);

				if (x_robot < 0 + 5) move_backward(pw_l, pw_r, ep, ei);
				if (x_robot > width - 1 - 5) move_backward(pw_l, pw_r, ep, ei);
				if (y_robot < 0 + 5) move_backward(pw_l, pw_r, ep, ei);
				if (y_robot > height - 1 - 5) move_backward(pw_l, pw_r, ep, ei);


				set_inputs(pw_l, pw_r, pw_laser, laser,
					light, light_gradient, light_dir, image_noise,
					max_speed, opponent_max_speed);

				///// FOR PLAYER MODE
				// NOTE: only one program can call view_image()
				view_rgb_image(rgb0);

				tc = high_resolution_time();

				//Process the new frame
				process_image(a, b, rgb, rgb0, rgb_a, rgb_b, label, tc, tc0, hist, nhist, hmin, hmax, nlabels);

				// Work not continued for centroid recovery beaausing leaving the space would be forfeit according to rules
				//filter_objects_check(x_robot, y_robot, d_SF, nlabels_actual, x_opp, y_opp, x_obs_array, y_obs_array, nlabels, label, x_size1, x_size2, y_size1, y_size2, b, rgb0, c1, c2, c3, c4, robot_a, robot_b, obstacles, obstacles_size, size_r, size_o);

				Sleep(10); // 100 fps max

		}
		

	// free the image memory before the program completes

	free_images(a, b, rgb, rgb0, rgb_a, rgb_b, label, mask, rgb_mask, edge_mag, edge_theta);

	deactivate_vision();
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}


int process_image(image &a, image &b, image &rgb, image &rgb0, image &rgb_a, image &rgb_b, image &label, double &tc, double &tc0, double *hist, int &nhist, double &hmin, double &hmax, int &nlabels){

	int j;
	double x;

	// simulates the robots and acquires the image from simulation
	acquire_image_sim(rgb);
	tc = high_resolution_time() - tc0;
	copy(rgb, rgb0);

	// scale the image to enhance contrast
	scale(rgb, rgb_a);
	// apply a filter to reduce noise
	//gaussion filter special property that two passes of this
	// filter is equivalent to one 5x5 guasian filter, and so on.
	// this filter is more accurate than the
	// averaging low pass filter, but you may need to 
	// apply it more often since their is higher weighting
	// to the center pixel.
	rgb_gaussian_filter(rgb_a, rgb_b);
	rgb_gaussian_filter(rgb_b, rgb_a);
	copy(rgb_a, a); // covert to Grey Scale

	/*
	// make a histogram
	nhist = 60; // make 60 bins -- each bin is 255/60 range of intensity
	// eg bin1 = 0-3
	// bin2 = 4-8,
	// etc.
	histogram(a, hist, nhist, hmin, hmax);

	// save to a csv file you can open/plot with microsoft excel
	// make sure the file is not currently open in excel before saving
	ofstream fout("hist1.csv");

	for (j = 0; j < nhist; j++) {
	x = hmin + (hmax - hmin) / nhist*j;
	fout << x << "," << hist[j] << "\n";
	}

	fout.close();
	*/


	//cout << "\n Look at Histogram and enter threshold value";
	//cin >> th;
	// use threshold function to make a binary image (0,255)
	threshold(a, b, 70);
	// invert the image
	invert(b, a);
	// perform an erosion function to remove noise (small objects)
	erode(a, b);
	copy(b, a); // New Line 
	erode(a, b); // New Line - 200% Obstacles from Irfan arent that good
	// perform a dialation function to fill in 
	// and grow the objects
	dialate(b, a);
	dialate(a, b);
	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(b, label, nlabels);

//	copy(a, rgb);
	//view_rgb_image(rgb);
	//pause();

	return 0;
}

int set_images(image &a, image &b, image &rgb, image &rgb0, image &rgb_a, image &rgb_b, image &label, image &mask, image &rgb_mask, image &edge_mag, image &edge_theta, int width, int height, int mask_width, int mask_height)

{
	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	rgb0.type = RGB_IMAGE;
	rgb0.width = width;
	rgb0.height = height;

	rgb_a.type = RGB_IMAGE;
	rgb_a.width = width;
	rgb_a.height = height;

	a.type = GREY_IMAGE;
	a.width = width;
	a.height = height;

	rgb_b.type = RGB_IMAGE;
	rgb_b.width = width;
	rgb_b.height = height;

	b.type = GREY_IMAGE;
	b.width = width;
	b.height = height;

	label.type = LABEL_IMAGE;
	label.width = width;
	label.height = height;

	mask.type = GREY_IMAGE;
	mask.width = mask_width;
	mask.height = mask_height;

	rgb_mask.type = RGB_IMAGE;
	rgb_mask.width = mask_width;
	rgb_mask.height = mask_height;

	edge_mag.type = GREY_IMAGE;
	edge_mag.width = mask_width;
	edge_mag.height = mask_height;

	edge_theta.type = GREY_IMAGE;
	edge_theta.width = mask_width;
	edge_theta.height = mask_height;

	return 0;
}

int allocate_images(image &a, image &b, image &rgb, image &rgb0, image &rgb_a, image &rgb_b, image &label, image &mask, image &rgb_mask, image &edge_mag, image &edge_theta)
{
	allocate_image(a);
	allocate_image(b);
	allocate_image(rgb_a);
	allocate_image(rgb_b);
	allocate_image(label);
	allocate_image(rgb);
	allocate_image(rgb0);
	allocate_image(mask);
	allocate_image(rgb_mask);
	allocate_image(edge_theta);
	allocate_image(edge_mag);

	return 0;
}

int free_images(image &a, image &b, image &rgb, image &rgb0, image &rgb_a, image &rgb_b, image &label, image &mask, image &rgb_mask, image &edge_mag, image &edge_theta)
{

	free_image(rgb);
	free_image(a);
	free_image(b);
	free_image(rgb_a);
	free_image(rgb_b);
	free_image(label);
	free_image(rgb0);
	free_image(mask);
	free_image(rgb_mask);
	free_image(edge_theta);
	free_image(edge_mag);
	return 0;
}