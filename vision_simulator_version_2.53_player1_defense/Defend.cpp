////
//FINAL VERSION <-- Defend (cpp)
//TEAM DeathStar (MECH 6631 - Robot Light Wars)
//		||Members
//		||Chandan Satija
//		||Tasnim Haque
//		||Farshad Zaboli
//		||Mohammad Saeidi

#include <iostream>
#include <cmath>
#include <Windows.h>

using namespace std;

// include this header file for basic image transfer functions
#include "image_transfer5.h"

// include this header file for computer vision functions
#include "vision.h"

//include header file for color functions
#include "DeathStar.h"

//include header file for defend functions
#include "Defend.h"



// main defend fn
int defend(int obs_number, int *x_obs_array, int *y_obs_array, int *obs_size, double &xgoal, double &ygoal, double &phi_goal, int x_robot, int y_robot, double phi_robot, int x_opp, int y_opp, double phi_opp){
	//bounded:-- there's enough space to allow robot to circle, w/in bounded area --> stalemate
	//unbounded:-- outside of bounded area, lucky --> park, else park & oscillate till death

	//local variables
	int l_robot = 100, w_robot = 85;
	int dim_obs_min = 65, dim_obs_max = 65 * 2;
	
	//assuming there is never an instance where we have two obstacles of different sizes
	int d_obs1=obs_size[1] , d_obs2=obs_size[2];
	//cout << "\nd_obs1 " << d_obs1 << "\nd_obs2" << d_obs2;
	int d_obs; //= dim_obs_min;
	if (obs_number == 1 || d_obs2 == (-1)){ d_obs = d_obs1; }
	else if (d_obs1 == d_obs2){ d_obs = d_obs2; }
	
	int d_SF = 5;
	//double error_tolerance = 3.0;
	int i;


	//calculate num_bounded
	int obs_number_b = 0;
	int obs_b_array[2 + 1], x_obs_b_array[2 + 1], y_obs_b_array[2 + 1], obs_d_array[2+1];
	int b_value = l_robot + (int)round(d_obs / 2.0) + d_SF;
	int x_b_min = 0 + b_value,	x_b_max = 640 - b_value,	y_b_min = 0 + b_value,	y_b_max = 480 - b_value;

	//cout << "\nTeam DeathStar defence mode activated";
	//cout << "\nxobs " << x_obs_array[1]; cout << "\tyobs " << y_obs_array[1];
	//cout << "\nxobs " << x_obs_array[2]; cout << "\tyobs " << y_obs_array[2];
	calc_obs_bounded(obs_number_b,obs_number,x_obs_array,y_obs_array,obs_b_array,x_obs_b_array,y_obs_b_array,x_b_min,x_b_max,y_b_min,y_b_max);
	//cout << "\nxobs " << x_obs_b_array[1]; cout << "\tyobs " << y_obs_b_array[1];
	//cout << "\nxobs " << x_obs_b_array[2]; cout << "\tyobs " << y_obs_b_array[2];

		
	//--scenario 1
	int x_obs_near, y_obs_near, num_near = 0;			
		
	//--scenario 2
	int x_obs_b_1, y_obs_b_1, x_obs_b_2, y_obs_b_2;		
	
	//--scenario 3
	int x_obs_ub, y_obs_ub; 
	double sqf;
	int side;

	//--scenario 4
	int x_obs_ub_1, y_obs_ub_1, x_obs_ub_2, y_obs_ub_2;
	
	//--scenario 5
	//int x_obs_ub, y_obs_ub, 
	int x_obs_b, y_obs_b;
	
	if (obs_number == 1 && obs_number_b == 1){
		//scenario 1 --> centered 1 obstacles
		for (i = 1; i <= obs_number; i++){ if (obs_b_array[i] > 0) break; }
		num_near = i;
		x_obs_near = x_obs_b_array[num_near];		y_obs_near = y_obs_b_array[num_near];

		//cout << "\nxobs " << x_obs_near; cout << "\tyobs " << y_obs_near;
		//cout << "\nentering scenario 1";
		defend_s1(xgoal, ygoal, phi_goal, x_obs_near, y_obs_near, x_opp, y_opp, phi_opp, d_obs, d_SF);
	}
	else if (obs_number == 1 && obs_number_b == 0){
		//scenario 3 --> bordered 1 obstacle
		for (i = 1; i <= obs_number; i++){ if (obs_b_array[i] == 0) break; }
		x_obs_ub = x_obs_array[i];			y_obs_ub = y_obs_array[i];

		//cout << "\nxobs " << x_obs_ub; cout << "\tyobs " << y_obs_ub;
		//cout << "\nentering scenario 3";
		defend_s3(xgoal, ygoal, phi_goal, sqf, side, x_obs_ub, y_obs_ub, x_opp, y_opp, phi_opp, x_robot, y_robot, phi_robot, d_obs, d_SF, w_robot, l_robot);
	}
	else if (obs_number == 2 && obs_number_b == 0){
		//scenario 4
		x_obs_ub_1 = x_obs_array[1];		y_obs_ub_1 = y_obs_array[1];
		x_obs_ub_2 = x_obs_array[2];		y_obs_ub_2 = y_obs_array[2];

		//cout << "\nxobs1 " << x_obs_ub_1; cout << "\tyobs1 " << y_obs_ub_1;
		//cout << "\nxobs2 " << x_obs_ub_2; cout << "\tyobs2 " << y_obs_ub_2;
		//cout << "\nentering scenario 4";

		defend_s4(xgoal, ygoal, phi_goal, x_obs_ub_1, y_obs_ub_1, x_obs_ub_2, y_obs_ub_2, x_opp, y_opp, phi_opp, x_robot, y_robot, phi_robot, d_obs, d_SF, w_robot, l_robot);
	}
	else if (obs_number == 2 && obs_number_b == 2){
		//scenario 2
		x_obs_b_1 = x_obs_array[1];		y_obs_b_1 = y_obs_array[1];
		x_obs_b_2 = x_obs_array[2];		y_obs_b_2 = y_obs_array[2];

		//cout << "\nxobs1 " << x_obs_b_1; cout << "\tyobs1 " << y_obs_b_1;
		//cout << "\nxobs2 " << x_obs_b_2; cout << "\tyobs2 " << y_obs_b_2;
		//cout << "\nentering scenario 2";
		defend_s2(xgoal, ygoal, phi_goal, x_obs_b_1, y_obs_b_1, x_obs_b_2, y_obs_b_2, x_opp, y_opp, phi_opp, d_obs, d_SF);
	}
	else if (obs_number == 2 && obs_number_b == 1){
		//scenario 5
		if (obs_number > obs_number_b){
			if (obs_b_array[1] == 0){		x_obs_ub = x_obs_array[1]; y_obs_ub = y_obs_array[1]; x_obs_b = x_obs_array[2]; y_obs_b = y_obs_array[2]; }
			else if (obs_b_array[2] == 0){	x_obs_ub = x_obs_array[2]; y_obs_ub = y_obs_array[2]; x_obs_b = x_obs_array[1]; y_obs_b = y_obs_array[1]; }
		}

		//cout << "\nxobs1 " << x_obs_ub;	cout << "\tyobs1 " << y_obs_ub;
		//cout << "\nxobs2 " << x_obs_b;	cout << "\tyobs2 " << y_obs_b;
		//cout << "\nentering scenario 5";
		defend_s5(xgoal, ygoal, phi_goal, x_obs_b, y_obs_b, x_obs_ub, y_obs_ub, x_opp, y_opp, phi_opp, x_robot, y_robot, phi_robot, d_obs, d_SF, l_robot, w_robot);

	}
	

	return 0;
}

//scenario 1	1 obs, 1 bounded
int defend_s1(double &xgoal, double &ygoal, double &phi_goal, int &x_obs_near, int &y_obs_near, int &x_opp, int &y_opp, double &phi_opp, int &d_obs, int &d_SF){
	
	//local variables
	//cout << "\nxobs " << x_obs_near; cout << "\tyobs " << y_obs_near;
	int b_value = round(d_obs / 2.0) + d_SF; //cout << "\nb_value " << b_value;
	double y_b_track_max, y_b_track_min, x_b_track_max, x_b_track_min;
	y_b_track_max = y_obs_near + b_value; //cout << "\nymax " << y_b_track_max;
	y_b_track_min = y_obs_near - b_value; //cout << "\tymin " << y_b_track_min;
	x_b_track_max = x_obs_near + b_value; //cout << "\txmax " << x_b_track_max;
	x_b_track_min = x_obs_near - b_value; //cout << "\txmin " << x_b_track_min;

	double inc_angle = 0.0;
	//		  2
	//	  12______32
	//	1  |	  |  3
	//	   |______|
	//    14	  34
	//		   4

	//calculate safe location 
	//1
	if (x_opp <= x_b_track_min){
		xgoal = (double)x_b_track_max;
		//corner 14
		if (y_opp <= y_b_track_min){  ygoal = (double)y_b_track_max;  }
		//corner 12
		else if (y_opp >= y_b_track_max) {	ygoal = (double)y_b_track_min; }
		else{ 
			//along 1
			if (y_opp<y_obs_near)	ygoal = (double)y_b_track_max;			//below y of obstacles centroid
			else if (y_opp>y_obs_near) ygoal = (double)y_b_track_min;		//above y
			else if (y_opp == y_obs_near) ygoal = (double)y_obs_near;		//at y
		}
	}
	//3
	else if (x_opp >= x_b_track_max){
		xgoal = (double)x_b_track_min;
		//corner 34
		if (y_opp <= y_b_track_min){ ygoal = (double)y_b_track_max; }
		//corner 32
		else if (y_opp >= y_b_track_max) { ygoal = (double)y_b_track_min; }
		else{ 
			//along 3
			if (y_opp<y_obs_near)	ygoal = (double)y_b_track_max;			//below y of obstacle
			else if (y_opp>y_obs_near) ygoal = (double)y_b_track_min;		//above y
			else if (y_opp == y_obs_near) ygoal = (double)y_obs_near;		//at y
		}
	}
	else{
		//4
		if (y_opp <= y_b_track_min) { 
			//along 4
			ygoal = (double)y_b_track_max; 
			if (x_opp<x_obs_near)	xgoal = (double)x_b_track_max;			//passed x of obstacle
			else if (x_opp>x_obs_near) xgoal = (double)x_b_track_min;		//before x
			else if (x_opp == x_obs_near) xgoal = (double)x_obs_near;		//at x
		}
		//2
		else if (y_opp >= y_b_track_max) { 
			//along 2
			ygoal = (double)y_b_track_min;									
			if (x_opp<x_obs_near)	xgoal = (double)x_b_track_max;			//passed x of obstacles
			else if (x_opp>x_obs_near) xgoal = (double)x_b_track_min;		//before x
			else if (x_opp == x_obs_near) xgoal = (double)x_obs_near;		//at x
		}
	}

	//calculate safe orientation
	calc_obj_orientation_2_xy(x_opp, y_opp, phi_opp, x_obs_near, y_obs_near, inc_angle); 
	phi_goal = phi_opp + inc_angle;

	return 0;
}

//scenario 2	2 obs, 2 bounded
int defend_s2(double &xgoal, double &ygoal, double &phi_goal, int &x_obs_b_1, int &y_obs_b_1, int &x_obs_b_2, int &y_obs_b_2, int &x_opp, int &y_opp, double &phi_opp, int &d_obs, int &d_SF){

	//local variables
	int b_value, b_value_l, b_value_w;
	double cx1, cx2, cx3, cx4;
	double cy1, cy2, cy3, cy4;
	double l1, l2, l3, l4;

	double inc_angle = 0.0;

	////TODO --> box of complicated shape to make circling more precise
	////calculate slope b/w two obstacles
	//double m, phi_obs, m_2_xy, m_c_opp;
	//double d_2_xy, d_hw, d_hl;
	//double cx, cy;
	//m = (double) (y_obs_b_2 - y_obs_b_1) / (x_obs_b_2 - x_obs_b_1);
	////m>0.0
	//m_2_xy = m;
	//cx = round((x_obs_b_1 + x_obs_b_2) / 2.0);	cy = round((y_obs_b_1 + y_obs_b_2) / 2.0);
	//m_c_opp = (double)(y_opp - cy) / (x_opp - cx);
	
	//d_2_xy = sqrt((double)(x_obs_b_2 - x_obs_b_1)*(x_obs_b_2 - x_obs_b_1) + (double)(y_obs_b_2 - y_obs_b_1)*(y_obs_b_2 - y_obs_b_1));
	////d_2_xy = round(d_2_xy);
	//d_hw = round(d_obs / 2.0) + d_SF;
	//d_hl = round(d_2_xy / 2.0) + d_hw;
	
	

	
	int xmin, xmax, ymin, ymax;
	int xmid, ymid;
	int xdim, ydim;

	//simplify to bigger box centered at a midpoint location determined from the two obstacles <-- pretending to be a very large obstacle
	calc_obs_outer_box(xmin, xmax, ymin, ymax, x_obs_b_1,y_obs_b_1,x_obs_b_2,y_obs_b_2,d_obs);
	xmid = round((xmin + xmax) / 2.0);		ymid = round((ymin + ymax) / 2.0);
	xdim = xmax - xmin;						ydim = ymax - ymin;

	//pick the "obstacle dimension"
	if (xdim >= ydim){		defend_s1(xgoal, ygoal, phi_goal, xmid, ymid, x_opp, y_opp, phi_opp, xdim, d_SF); }
	else if (xdim < ydim){	defend_s1(xgoal, ygoal, phi_goal, xmid, ymid, x_opp, y_opp, phi_opp, ydim, d_SF); }
	

	
	return 0;
}

//scenario 3	1 obs, 1 unbounded
int defend_s3(double &xgoal, double &ygoal, double &phi_goal, double &sqf, int &side, int &x_obs, int &y_obs, int &x_opp, int &y_opp, double &phi_opp, int &x_robot, int &y_robot, double &phi_robot, int &d_obs, int &d_SF, int &d_rw, int &d_rl){
	//purpose: to find optimal position to park, when circling is impossible
	
	//calculate some spacing required 
	double d_req_r_w = d_rw + 2 * d_SF, d_req_r_w_center = (int)round(d_req_r_w / 2.0);
	double sqf_req = d_rw + d_SF; //+ round(d_obs / 2.0);
	double d_req_obs_r = round(d_obs / 2.0) + d_req_r_w_center;

	//calculate the boundary area that allows for circling, being outside, means obstacle is along edges of field/space
	double x_b_min, x_b_max, y_b_min, y_b_max;
	int b_value = (int)(d_rl + round(d_obs / 2.0) + d_SF);
	//1						3							4						2
	x_b_min = 0 + b_value;	x_b_max = 640 - b_value;	y_b_min = 0 + b_value;	y_b_max = 480 - b_value;
	
	//when unsure, of which location to pick --> send to fn that will pick correct option based on distance to be covered
	double opx1, opy1, opth1, opx2, opy2, opth2, distrobot1, distrobot2;
	double distop1, distop2;
	double sqf1, sqf2;
	int side1, side2;

	//squish factor, to be used to look for tighter space to fit into
	double sqfy2, sqfy4, sqfx1, sqfx3;
	//space available to park in, from all side of obstacles
	sqfy2 = 480 - (y_obs + round(d_obs / 2.0) + d_SF);			sqfx3 = 640 - (x_obs + round(d_obs / 2.0) + d_SF);
	sqfy4 = (y_obs - round(d_obs / 2.0) - d_SF) - 0;			sqfx1 = (x_obs - round(d_obs / 2.0) - d_SF) - 0;


	//8 cases		corners 12, 14, 32, 34		sides 1, 2, 3, 4
	//		  2
	//	  12______32
	//	1  |	  |  3
	//	   |______|
	//    14	  34
	//		   4


	//1
	if (x_obs <= x_b_min){			
		//check if corner 14	--> park
		if (y_obs <= y_b_min){
			opx1 = x_obs + d_req_obs_r;		opy1 = 0 + d_SF;			opth1 = -90.0;		sqf1 = sqfx3;	side1 = 4;
			opx2 = 0 + d_SF;				opy2 = y_obs + d_req_obs_r;	opth2 = 180.0;		sqf2 = sqfy2;	side2 = 1;
			calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2,side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
			
		}
		//check if corner 12	--> park
		else if (y_obs >= y_b_max){
			opx1 = x_obs + d_req_obs_r;		opy1 = 480 - d_SF;			opth1 = 90.0;	sqf1 = sqfx3;	side1 = 2;
			opx2 = 0 + d_SF;				opy2 = y_obs - d_req_obs_r;	opth2 = 180.0;	sqf2 = sqfy4;	side2 = 1;
			calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
		}
		//check along side 1
		else {
			xgoal = 0 + d_SF;	phi_goal = 180.0;

			//check tighter space
			//closer to side 2
			if (sqfy2<sqfy4){
				if (sqf_req <= sqfy2){	ygoal = y_obs + d_req_obs_r;	sqf = sqfy2;	side = 1; }
				else{					ygoal = y_obs - d_req_obs_r;	sqf = sqfy4;	side = 1; }
			}
			//closer to side 4
			else if (sqfy2>sqfy4){
				if (sqf_req <= sqfy4){	ygoal = y_obs - d_req_obs_r;	sqf = sqfy4;	side = 1; }
				else{					ygoal = y_obs + d_req_obs_r;	sqf = sqfy2;	side = 1; }
			}
			//equal spacing
			else if (sqfy2==sqfy4){
				opx1 = xgoal;		opy1 = y_obs + d_req_obs_r;		opth1 = phi_goal;	sqf1 = sqfy2;	side1 = 1;
				opx2 = xgoal;		opy2 = y_obs - d_req_obs_r;		opth2 = phi_goal;	sqf2 = sqfy4;	side2 = 1;
				calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
			}
			

		}
	}
	//3
	if (x_obs >= x_b_max){
		//check if corner 34	--> park
		if (y_obs <= y_b_min){
			opx1 = x_obs - d_req_obs_r;		opy1 = 0 + d_SF;			opth1 = -90.0;	sqf1 = sqfx1;	side1 = 4;
			opx2 = 640 - d_SF;				opy2 = y_obs + d_req_obs_r;	opth2 = 0.0;	sqf2 = sqfy2;	side2 = 3;
			calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
		}
		//check if corner 32	--> park
		else if (y_obs >= y_b_max){
			opx1 = x_obs - d_req_obs_r;		opy1 = 480 - d_SF;			opth1 = 90.0;	sqf1 = sqfx1;	side1 = 2;
			opx2 = 640 - d_SF;				opy2 = y_obs - d_req_obs_r;	opth2 = 0.0;	sqf2 = sqfy4;	side2 = 3;
			calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
		}
		//check along side 3
		else {
			xgoal = 640 - d_SF;	phi_goal = 0.0;
			
			//check for tighter space
			//closer to side 2
			if (sqfy2<sqfy4){
				if (sqf_req <= sqfy2){	ygoal = y_obs + d_req_obs_r;	sqf = sqfy2;	side = 3; }
				else{					ygoal = y_obs - d_req_obs_r;	sqf = sqfy4;	side = 3; }
			}
			//closer to side 4
			else if (sqfy2>sqfy4){
				if (sqf_req <= sqfy4){	ygoal = y_obs - d_req_obs_r;	sqf = sqfy4;	side = 3; }
				else{					ygoal = y_obs + d_req_obs_r;	sqf = sqfy2;	side = 3; }
			}
			//equal spacing
			else if (sqfy2==sqfy4){
				opx1 = xgoal;		opy1 = y_obs + d_req_obs_r;			opth1 = phi_goal;		sqf1 = sqfy2;	side1 = 3;
				opx2 = xgoal;		opy2 = y_obs - d_req_obs_r;			opth2 = phi_goal;		sqf2 = sqfy4;	side2 = 3;
				calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
			}
		}
	}
	//4
	if (y_obs <= y_b_min){
		//check along side 4

		ygoal = 0 + d_SF;	phi_goal = -90.0;
		
		//closer to side 3
		if(sqfx3<sqfx1){
			if (sqf_req <= sqfx3){	xgoal = x_obs + d_req_obs_r;	sqf = sqfx3;	side = 4; }
			else{					xgoal = x_obs - d_req_obs_r;	sqf = sqfx1;	side = 4; }
		}
		//closer to side 1
		else if(sqfx3 > sqfx1) {
			if (sqf_req <= sqfx1){	xgoal = x_obs - d_req_obs_r;	sqf = sqfx1;	side = 4; }
			else{					xgoal = x_obs + d_req_obs_r;	sqf = sqfx3;	side = 4; }
		}
		//equal spacing
		else if(sqfx3 == sqfx1 ){
			opx1 = x_obs + d_req_obs_r;		opy1 = ygoal;			opth1 = phi_goal;		sqf1 = sqfx3;	side1 = 4;
			opx2 = x_obs - d_req_obs_r;		opy2 = ygoal;			opth2 = phi_goal;		sqf2 = sqfx1;	side2 = 4;
			calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
		}
		
	}
	//2
	if (y_obs >= y_b_max){
		//along side 2

		ygoal = 480 - d_SF;	phi_goal = 90.0;
		
		//closer to side 3
		if (sqfx3<sqfx1){
			if (sqf_req <= sqfx3){	xgoal = x_obs + d_req_obs_r;	sqf = sqfx3;	side = 2; }
			else{					xgoal = x_obs - d_req_obs_r;	sqf = sqfx1;	side = 2; }
		}
		//closer to side 1
		else if (sqfx3 > sqfx1) {
			if (sqf_req <= sqfx1){	xgoal = x_obs - d_req_obs_r;	sqf = sqfx1;	side = 2; }
			else{					xgoal = x_obs + d_req_obs_r;	sqf = sqfx3;	side = 2; }
		}
		//equal spacing
		else if (sqfx3 == sqfx1){
			opx1 = x_obs + d_req_obs_r;		opy1 = ygoal;			opth1 = phi_goal;		sqf1 = sqfx3;	side1 = 2;
			opx2 = x_obs - d_req_obs_r;		opy2 = ygoal;			opth2 = phi_goal;		sqf2 = sqfx1;	side2 = 2;
			calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

		}
	}



	return 0;
}

//scenario 4	2 obs, 2 unbounded
int defend_s4(double &xgoal, double &ygoal, double &phi_goal, int &xobs1, int &yobs1, int &xobs2, int &yobs2, int &x_opp, int &y_opp, double &phi_opp, int &x_robot, int &y_robot, double &phi_robot, int &d_obs, int &d_SF, int &d_rw, int &d_rl){

	//local variables
	double dist_r_obs_1, dist_r_obs_2, dist_r_opp;//
	double dist_opp_obs_1, dist_opp_obs_2;//
	double dist_obs, phi_obs;
	int closer_obs_robot = 0, further_obs_opp = 0;
	int ans, q, s, i;

	int flag_checkoppside = 0;

	int xminb, xmaxb, yminb, ymaxb;
	int xminouter, xmaxouter, yminouter, ymaxouter;
	int xmininner, xmaxinner, ymininner, ymaxinner;
	int innerdy, innerdx;
	int parkzones[3 + 1];
	double dist_b_req = d_rl + round(d_obs / 2.0) + d_SF;
	xminb = 0 + (int)dist_b_req;	xmaxb = 640 - (int)dist_b_req;		yminb = 0 + (int)dist_b_req;	ymaxb = 480 - (int)dist_b_req;

		//case1  
		double sqf_reqh, sqf_reqhx2;
		double sqfx1, sqfy2, sqfx3, sqfy4;
		double sqf_robot = d_rw + 2.0*d_SF;
		sqf_reqhx2 = d_rw*2.0 + d_obs + d_SF*2.0;
		sqf_reqh = round(sqf_reqhx2 / 2.0) + d_SF;
		int robotf = (int)round(sqf_robot / 2.0);
		
		//case 3
		double sqf;
		int side;
		double opx1, opy1, opth1, sqf1;		double opx2, opy2, opth2, sqf2;
		int side1;								int side2;
		//ideal option 1 <- obstacle 1		//ideal option 2 <- obstacle 2

		calc_dist(xobs1, yobs1, xobs2, yobs2, dist_obs);

		//1-- less than enough space
		if (dist_obs < sqf_reqh){
			//not enough space
			//calculate all options
			calc_obs_outer_box(xminouter, xmaxouter, yminouter, ymaxouter, xobs1, yobs1, xobs2, yobs2,d_obs);
			sqfx1 = xminouter - 0;		sqfx3 = 640 - xmaxouter;
			sqfy4 = yminouter - 0;		sqfy2 = 480 - ymaxouter;

			//8 cases		corners 12, 14, 32, 34		sides 1, 2, 3, 4
			//		  2
			//	  12______32
			//	1  |	  |  3
			//	   |______|
			//    14	  34
			//		   4

			//1
			if (sqfx1 < sqf_robot){
				//corner 14
				if (sqfy4 < sqf_robot){
					opx1 = xmaxouter + robotf;		opy1 = 0 + d_SF;				opth1 = -90.0;		sqf1 = sqfx3;	side1 = 4;
					opx2 = 0 + d_SF;				opy2 = ymaxouter + robotf;		opth2 = 180.0;		sqf2 = sqfy2;	side2 = 1;
					calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

				}
				//corner 12
				else if (sqfy2 < sqf_robot){
					opx1 = xmaxouter + robotf;		opy1 = 480 - d_SF;				opth1 = 90.0;		sqf1 = sqfx3;	side1 = 2;
					opx2 = 0 + d_SF;				opy2 = yminouter - robotf;		opth2 = 180.0;		sqf2 = sqfy4;	side2 = 1;
					calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
				}
				//side 1
					//compare sqf2, sqf4
				else{
					xgoal = 0 + d_SF;	phi_goal = 180.0;
					if (sqfy4 < sqfy2){
						if (sqf_robot <= sqfy4){	ygoal = yminouter - robotf; }
						else {						ygoal = ymaxouter + robotf; }
					}
					else if (sqfy4>sqfy2){
						if (sqf_robot <= sqfy2){	ygoal = ymaxouter + robotf; }
						else{						ygoal = yminouter - robotf; }
					}
					else if (sqfy4 == sqfy2){
						opx1 = xgoal;		opy1 = ymaxouter + robotf; 			opth1 = phi_goal;		sqf1 = sqfy2;		side1 = 1;
						opx2 = xgoal;		opy2 = yminouter - robotf;			opth2 = phi_goal;		sqf2 = sqfy4;		side2 = 1;
						calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

					}
				}
			}
			//3
			if (sqfx3 < sqf_robot){
				//corner 34
				if (sqfy4 < sqf_robot){
					opx1 = xminouter - robotf;		opy1 = 0 + d_SF;				opth1 = -90.0;		sqf1 = sqfx1;	side1 = 4;
					opx2 = 640 - d_SF;				opy2 = ymaxouter + robotf;		opth2 = 0.0;		sqf2 = sqfy2;	side2 = 3;
					calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

				}
				//corner 32
				else if (sqfy2 < sqf_robot){
					opx1 = xminouter - robotf;		opy1 = 480 - d_SF;				opth1 = 90.0;		sqf1 = sqfx1;	side1 = 2;
					opx2 = 640 - d_SF;				opy2 = yminouter - robotf;		opth2 = 0.0;		sqf2 = sqfy4;	side2 = 3;
					calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
				}
				//side 3 
					//compare sqf2, sqf4
				else{
					xgoal = 640 - d_SF;	phi_goal = 0.0;

					if (sqfy4 < sqfy2){
						if (sqf_robot <= sqfy4){	ygoal = yminouter - robotf; }
						else {						ygoal = ymaxouter + robotf; }
					}
					else if (sqfy4>sqfy2){
						if (sqf_robot <= sqfy2){	ygoal = ymaxouter + robotf; }
						else{						ygoal = yminouter - robotf; }
					}
					else if (sqfy4 == sqfy2){
						opx1 = xgoal;		opy1 = ymaxouter + robotf; 			opth1 = phi_goal;		sqf1 = sqfy2;		side1 = 3;
						opx2 = xgoal;		opy2 = yminouter - robotf;			opth2 = phi_goal;		sqf2 = sqfy4;		side2 = 3;
						calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

					}
				}
			}
			//4
			if (sqfy4 < sqf_robot){
				//side 4
					//comapre sqf1, sqf3
				ygoal = 0 + d_SF;	phi_goal = -90.0;

				if (sqfx1 < sqfx3){
					if (sqf_robot <= sqfx1){	xgoal = xminouter - robotf; }
					else {						xgoal = xmaxouter + robotf; }
				}
				else if (sqfx1>sqfx3){
					if (sqf_robot <= sqfx3){	xgoal = xmaxouter + robotf; }
					else{						xgoal = xminouter - robotf; }
				}
				else if (sqfx1 == sqfx3){
					opx1 = xmaxouter + robotf;		opy1 = xgoal;			opth1 = phi_goal;		sqf1 = sqfx3;		side1 = 4;
					opx2 = xminouter - robotf;		opy2 = xgoal;			opth2 = phi_goal;		sqf2 = sqfx1;		side2 = 4;
					calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

				}
			}
			//2
			if (sqfy2 < sqf_robot){
				//side 2
					//comapre sqf1, sqf3
				ygoal = 480 - d_SF;	phi_goal = 90.0;

				if (sqfx1 < sqfx3){
					if (sqf_robot <= sqfx1){	xgoal = xminouter - robotf; }
					else {						xgoal = xmaxouter + robotf; }
				}
				else if (sqfx1>sqfx3){
					if (sqf_robot <= sqfx3){	xgoal = xmaxouter + robotf; }
					else{						xgoal = xminouter - robotf; }
				}
				else if (sqfx1 == sqfx3){
					opx1 = xmaxouter + robotf;		opy1 = xgoal;			opth1 = phi_goal;		sqf1 = sqfx3;		side1 = 2;
					opx2 = xminouter - robotf;		opy2 = xgoal;			opth2 = phi_goal;		sqf2 = sqfx1;		side2 = 2;
					calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

				}
			}

			
		}
		//2-- just enough space
		else if (dist_obs >= sqf_reqh && dist_obs <= sqf_reqhx2){
				//park in b/w, if possible

				calc_obs_outer_box(xminouter, xmaxouter, yminouter, ymaxouter, xobs1, yobs1, xobs2, yobs2, d_obs);
				sqfx1 = xminouter - 0;		sqfx3 = 640 - xmaxouter;
				sqfy4 = yminouter - 0;		sqfy2 = 480 - ymaxouter;
				calc_obs_inner_box(xmininner, xmaxinner, ymininner, ymaxinner, xobs1, yobs1, xobs2, yobs2, d_obs);
				innerdx = xmaxinner - xmaxinner;		innerdy = ymaxinner - ymininner;

				//8 cases		corners 12, 14, 32, 34		sides 1, 2, 3, 4
				//		  2
				//	  12______32
				//	1  |	  |  3
				//	   |______|
				//    14	  34
				//		   4

				//1
				if (sqfx1 < sqf_robot){
					//corner 14
					if (sqfy4 < sqf_robot){
						if (innerdx < innerdy){
							if (sqf_robot <= innerdx){		xgoal = round((xmininner + xmaxinner) / 2.0);	ygoal = 0 + d_SF;								phi_goal = -90.0; }
							else {
								if (sqf_robot <= innerdy){	xgoal = 0 + d_SF;								ygoal = round((ymininner + ymaxinner) / 2.0);	phi_goal = +180.0; }
								else{
									opx1 = xmaxouter + robotf;		opy1 = 0 + d_SF;			opth1 = -90.0;		sqf1 = sqfx3;	side1 = 4;
									opx2 = 0 + d_SF;				opy2 = ymaxouter + robotf;	opth2 = 180.0;		sqf2 = sqfy2;	side2 = 1;
									calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

								}
							}
						}
						else if (innerdx>innerdy){
							if (sqf_robot <= innerdy){		xgoal = 0 + d_SF;								ygoal = round((ymininner + ymaxinner) / 2.0);	phi_goal = +180.0; }
							else {
								if (sqf_robot <= innerdx){	xgoal = round((xmininner + xmaxinner) / 2.0);	ygoal = 0 + d_SF;								phi_goal = -90.0; }
								else{
									opx1 = xmaxouter + robotf;		opy1 = 0 + d_SF;			opth1 = -90.0;		sqf1 = sqfx3;	side1 = 4;
									opx2 = 0 + d_SF;				opy2 = ymaxouter + robotf;	opth2 = 180.0;		sqf2 = sqfy2;	side2 = 1;
									calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
						
								}
							}
						}
						else if (innerdx == innerdy){
							opx1 = round((xmaxinner + xmaxinner) / 2.0); 		opy1 = 0 + d_SF;							opth1 = -90.0;		sqf1 = sqfx3;	side1 = 4;
							opx2 = 0 + d_SF;									opy2 = round((ymininner+ymaxinner)/2.0);	opth2 = 180.0;		sqf2 = sqfy2;	side2 = 1;
							calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

						}	
					}
					//corner 12
					else if (sqfy2 < sqf_robot){
									
							if (innerdx < innerdy){
								if (sqf_robot <= innerdx){		xgoal = round((xmininner + xmaxinner) / 2.0);	ygoal = 480- d_SF;								phi_goal = +90.0; }
								else {
									if (sqf_robot <= innerdy){	xgoal = 0 + d_SF;								ygoal = round((ymininner + ymaxinner) / 2.0);	phi_goal = +180.0; }
									else{
										opx1 = xmaxouter + robotf;		opy1 = 480 - d_SF;			opth1 = 90.0;		sqf1 = sqfx3;	side1 = 2;
										opx2 = 0 + d_SF;				opy2 = yminouter - robotf;	opth2 = 180.0;		sqf2 = sqfy4;	side2 = 1;
										calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

									}
								}
							}
							else if (innerdx>innerdy){
								if (sqf_robot <= innerdy){		xgoal = 0 + d_SF;								ygoal = round((ymininner + ymaxinner) / 2.0);	phi_goal = +180.0; }
								else {
									if (sqf_robot <= innerdx){	xgoal = round((xmininner + xmaxinner) / 2.0);	ygoal = 480 - d_SF;								phi_goal = +90.0; }
									else{
										opx1 = xmaxouter + robotf;		opy1 = 480 - d_SF;			opth1 = 90.0;		sqf1 = sqfx3;	side1 = 2;
										opx2 = 0 + d_SF;				opy2 = yminouter - robotf;	opth2 = 180.0;		sqf2 = sqfy4; side2 = 1;
										calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);
							
									}
								}
							}
							else if (innerdx == innerdy){
								opx1 = round((xmaxinner + xmaxinner) / 2.0); 		opy1 = 480 - d_SF;								opth1 = +90.0;		sqf1 = sqfx3;	side1 = 2;
								opx2 = 0 + d_SF;									opy2 = round((ymininner + ymaxinner) / 2.0);	opth2 = +180.0;		sqf2 = sqfy4;	side2 = 1;
								calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

							}
										
						
					}
					//side 1
					//compare sqf2, sqf4
					else{
						xgoal = 0 + d_SF;	phi_goal = 180.0;
						
						calc_order_park_inc(parkzones, sqfy2, innerdy, sqfy4);
						for (i = 1; i <= 3; i++){
							if (sqf_robot <= parkzones[i]){
								if (parkzones[i] == sqfy2)			ygoal = ymaxouter + robotf;
								else if (parkzones[i]==innerdy)		ygoal = round((ymaxinner+ymininner) / 2.0);
								else if (parkzones[i] == sqfy4)		ygoal = yminouter - robotf;
							}
						}
						
						
					}
				}
				//3
				if (sqfx3 < sqf_robot){
					//corner 34
					if (sqfy4 < sqf_robot){
						if (innerdx < innerdy){
							if (sqf_robot <= innerdx){		xgoal = round((xmininner + xmaxinner) / 2.0);	ygoal = 0 + d_SF;								phi_goal = -90.0; }
							else {
								if (sqf_robot <= innerdy){	xgoal = 640 - d_SF;								ygoal = round((ymininner + ymaxinner) / 2.0);	phi_goal = 0.0; }
								else{
									opx1 = xminouter - robotf;		opy1 = 0 + d_SF;			opth1 = -90.0;		sqf1 = sqfx1;	side1 = 4;
									opx2 = 640 - d_SF;				opy2 = ymaxouter + robotf;	opth2 = 0.0;		sqf2 = sqfy2;	side2 = 3;
									calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

								}
							}
						}
						else if (innerdx>innerdy){
							if (sqf_robot <= innerdy){		xgoal = 640 - d_SF;								ygoal = round((ymininner + ymaxinner) / 2.0);	phi_goal = 0.0; }
							else {
								if (sqf_robot <= innerdx){	xgoal = round((xmininner + xmaxinner) / 2.0);	ygoal = 0 + d_SF;								phi_goal = -90.0; }
								else{
									opx1 = xminouter - robotf;		opy1 = 0 + d_SF;			opth1 = -90.0;		sqf1 = sqfx1;	side1 = 4;
									opx2 = 640 - d_SF;				opy2 = ymaxouter + robotf;	opth2 = 0.0;		sqf2 = sqfy2;	side2 = 1;
									calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

								}
							}
						}
						else if (innerdx == innerdy){
							opx1 = round((xmaxinner + xmaxinner) / 2.0); 	opy1 = 0 + d_SF;								opth1 = -90.0;		sqf1 = sqfx1;	side1 = 4;
							opx2 = 640 - d_SF;								opy2 = round((ymininner + ymaxinner) / 2.0);	opth2 = 0.0;		sqf2 = sqfy2;	side2 = 3;
							calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

						}
					}
					//corner 32
					else if (sqfy2 < sqf_robot){

						if (innerdx < innerdy){
							if (sqf_robot <= innerdx){		xgoal = round((xmininner + xmaxinner) / 2.0);	ygoal = 480 - d_SF;								phi_goal = +90.0; }
							else {
								if (sqf_robot <= innerdy){	xgoal = 640 - d_SF;								ygoal = round((ymininner + ymaxinner) / 2.0);	phi_goal = 0.0; }
								else{
									opx1 = xminouter - robotf;		opy1 = 480 - d_SF;			opth1 = 90.0;		sqf1 = sqfx1;	side1 = 2;
									opx2 = 640 - d_SF;				opy2 = yminouter - robotf;	opth2 = 0.0;		sqf2 = sqfy4;	side2 = 3;
									calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

								}
							}
						}
						else if (innerdx>innerdy){
							if (sqf_robot <= innerdy){		xgoal = 640 - d_SF;								ygoal = round((ymininner + ymaxinner) / 2.0);	phi_goal = 0.0; }
							else {	
								if (sqf_robot <= innerdx){	xgoal = round((xmininner + xmaxinner) / 2.0);	ygoal = 480 - d_SF;								phi_goal = +90.0; }
								else{
									opx1 = xminouter - robotf;		opy1 = 480 - d_SF;			opth1 = +90.0;		sqf1 = sqfx1;	side1 = 2;
									opx2 = 640 - d_SF;				opy2 = yminouter - robotf;	opth2 = 0.0;		sqf2 = sqfy4;	side2 = 3;
									calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

								}
							}
						}
						else if (innerdx == innerdy){
							opx1 = round((xmaxinner + xmaxinner) / 2.0); 	opy1 = 480 - d_SF;								opth1 = +90.0;		sqf1 = sqfx1;	side1 = 2;
							opx2 = 640 - d_SF;								opy2 = round((ymininner + ymaxinner) / 2.0);	opth2 = 0.0;		sqf2 = sqfy4;	side2 = 3;
							calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp);

						}


					}
					//side 3
					//compare sqf2, sqf4
					else{
						xgoal = 640 - d_SF;	phi_goal = 0.0;

						calc_order_park_inc(parkzones, sqfy2, innerdy, sqfy4);
						for (i = 1; i <= 3; i++){
							if (sqf_robot <= parkzones[i]){
								if (parkzones[i] == sqfy2)			ygoal = ymaxouter + robotf;
								else if (parkzones[i] == innerdy)	ygoal = round((ymaxinner + ymininner) / 2.0);
								else if (parkzones[i] == sqfy4)		ygoal = yminouter - robotf;
							}
						}


					}
				}
				//4
				if (sqfy4 < sqf_robot){
					//side 4
					//comapre sqf1, sqf3
					ygoal = 0 + d_SF;	phi_goal = -90.0;

					calc_order_park_inc(parkzones, sqfx1, innerdx, sqfx3);
					for (i = 1; i <= 3; i++){
						if (sqf_robot <= parkzones[i]){
							if (parkzones[i] == sqfx3)			xgoal = xmaxouter + robotf;
							else if (parkzones[i] == innerdx)	xgoal = round((xmaxinner + xmininner) / 2.0);
							else if (parkzones[i] == sqfx1)		xgoal = xminouter - robotf;
						}
					}

				}
				//2
				if (sqfy2 < sqf_robot){
					//side 2
					//comapre sqf1, sqf3
					ygoal = 480 - d_SF;	phi_goal = 90.0;

					calc_order_park_inc(parkzones, sqfx1, innerdx, sqfx3);
					for (i = 1; i <= 3; i++){
						if (sqf_robot <= parkzones[i]){
							if (parkzones[i] == sqfx3)			xgoal = xmaxouter + robotf;
							else if (parkzones[i] == innerdx)	xgoal = round((xmaxinner + xmininner) / 2.0);
							else if (parkzones[i] == sqfx1)		xgoal = xminouter - robotf;
						}
					}


				}

				

			}
	else if (dist_obs>sqf_reqhx2){
			//call scenario 3 for each obstacle, and go with best "squish factor" <-- minimal space to park into
			defend_s3(opx1, opy1, opth1, sqf1, side1, xobs1, yobs1, x_opp, y_opp, phi_opp, x_robot, y_robot, phi_robot, d_obs, d_SF, d_rw, d_rl);
			defend_s3(opx2, opy2, opth2, sqf2, side2, xobs2, yobs2, x_opp, y_opp, phi_opp, x_robot, y_robot, phi_robot, d_obs, d_SF, d_rw, d_rl);

			if (sqf1 < sqf2){		xgoal = opx1; ygoal = opy1; phi_goal = opth1; }
			else if (sqf1>sqf2){	xgoal = opx2; ygoal = opy2; phi_goal = opth2; }
			else if (sqf1 == sqf2){ calc_best_option(opx1, opy1, opth1, sqf1, side1, opx2, opy2, opth2, sqf2, side2, xgoal, ygoal, phi_goal, sqf, side, x_robot, y_robot, x_opp, y_opp); }
		}



	
	return 0;
}

//scenario 5	2 obs, 1 bounded, 1 unbounded
int defend_s5(double &xgoal, double &ygoal, double &phi_goal, int &x_obs_b_1, int &y_obs_b_1, int &x_obs_ub_2, int &y_obs_ub_2, int &x_opp, int &y_opp, double &phi_opp, int &x_robot, int &y_robot, double &phi_robot, int &d_obs, int &d_SF, int &l_robot, int &w_robot){
	//variables
	int ans, q, s;
	double b_value = d_SF + round(d_obs / 2.0) + l_robot;
	double ub_value = d_SF + round(d_obs / 2.0);

	double x_r_b_max, x_r_b_min, y_r_b_max, y_r_b_min;
	double x_obs_ub_max, x_obs_ub_min, y_obs_ub_max, y_obs_ub_min;
	int flagx_clear=0, flagy_clear=0;

	x_r_b_max = x_obs_b_1 + b_value;	x_r_b_min = x_obs_b_1 - b_value;	y_r_b_max = y_obs_b_1 + b_value;	y_r_b_min = y_obs_b_1 - b_value;
	x_obs_ub_max = x_obs_ub_2 + ub_value;	x_obs_ub_min = x_obs_ub_2 - ub_value;	y_obs_ub_max = y_obs_ub_2 + ub_value;	y_obs_ub_min = y_obs_ub_2 - ub_value;

	//if circling possible --> defend_s1
	if (x_obs_ub_2 < x_obs_b_1){	if (x_obs_ub_max <= x_r_b_min)	flagx_clear = 1;	}
	else if (x_obs_ub_2>x_obs_b_1){	if (x_obs_ub_min >= x_r_b_max)	flagx_clear = 1;	}
	
	if (y_obs_ub_2 < y_obs_b_1){	if (y_obs_ub_max <= y_r_b_max)	flagy_clear = 1;	}
	else if (y_obs_ub_2>y_obs_b_1){	if (y_obs_ub_min >= y_r_b_max)	flagy_clear = 1;	}

	if (flagx_clear == 1 && flagy_clear == 1){ 
		//attempt circling
		//cout << "\nentering scenario 1 from scenario 5";
		defend_s1(xgoal, ygoal, phi_goal, x_obs_b_1, y_obs_b_1, x_opp, y_opp, phi_opp, d_obs, d_SF); 
	}
	else{
		//attempt parking --> scenario similar to scenario 3, just more complicated sicne we've got a blob of obstacles
		//cout << "\nentering scenario 4 from scenario 5";
		defend_s4(xgoal,ygoal,phi_goal,x_obs_b_1,y_obs_b_1,x_obs_ub_2,y_obs_ub_2,x_opp,y_opp,phi_opp,x_robot,y_robot,phi_robot,d_obs,d_SF,w_robot,l_robot);


	}



	





			

	return 0;
}



//beginning of helper functions
//calculate the number of bounded obstacles
int calc_obs_bounded(int &obs_number_b, int &obs_number, int *x_obs_array, int *y_obs_array,int *obs_b, int *x_obs_b, int *y_obs_b, int x_b_min, int x_b_max, int y_b_min, int y_b_max){
	
	//local vairables
	int flag_x = 0, flag_y = 0;
	int i;

	for (i = 1; i <= obs_number; i++){
		if (x_obs_array[i] >= x_b_min && x_obs_array[i] <= x_b_max) flag_x = 1;
		if (y_obs_array[i] >= y_b_min && y_obs_array[i] <= y_b_max) flag_y = 1;

		if (flag_x == 1 && flag_y == 1){
			obs_b[i] = i;
			x_obs_b[i] = x_obs_array[i];
			y_obs_b[i] = y_obs_array[i];
			obs_number_b++;
		}
		else{
			obs_b[i] = 0;
			x_obs_b[i] = 0;
			y_obs_b[i] = 0;
		}

		//cout << "\ni" << i <<"\tobs" << obs_bounded[i] << "\tx" << x_obs_bounded[i] << "\ty" << y_obs_bounded[i];
		flag_x = 0;
		flag_y = 0;
	}

	return 0;
}

//fn to check whether orientation towards a specific x,y
int calc_obj_orientation_2_xy(int x_opp, int y_opp, double phi_opp, int x_obs, int y_obs, double &inc_angle){

	if(phi_opp <= 180.0 && phi_opp > 0)			inc_angle = -180.0;
	else if (phi_opp > -180.0 && phi_opp <= 0)	inc_angle = +180.0;

	return 0;
}

//fn to calculate which quadrant & sector <-- final vers (UNUSED)
int calc_obj_quadrant_sector(int x, int y, int &q, int &s){
	
	//local variables
	int xmin = 0, xmax = 640, ymin = 0, ymax = 480;
	int cx = (xmin + xmax) / 2, cy = (ymin + ymax) / 2;

	//quad
	if (x > cx && y > cy) {
		q = 1;
		if ((y - x) < 0)		s = 1;
		else if ((y - x)>0)		s = 2;
		else if ((y - x) == 0)	s = 12;
	}
	else if (x<cx && y>cy) {
		q = 2; 
		if ((y + x) < 0)		s = 4;
		else if ((y + x) > 0)	s = 3;
		else if ((y - x) == 0)	s = 34;
	}
	else if (x < cx && y < cy) {
		q = 3; 
		if ((y - x) < 0)		s = 5;
		else if ((y - x)>0)		s = 6;
		else if ((y - x) == 0)	s = 56;
	}
	else if (x > cx && y < cy) {
		q = 4;
		if ((y + x) < 0)		s = 7;
		else if ((y + x) > 0)	s = 8;
		else if ((y - x) == 0)	s = 78;
	}
	else if (x == cy && y > cy) q = 12;
	else if (x == cy && y < cy) q = 34;
	else if (x > cx && y == cy) q = 41;
	else if (x < cx && y == cy) q = 23;
	

	return 0;
}

//fn to check whether two points are in opposite quadrants <-- final vers (UNUSED)
int check_quadrant_opp(int x1, int y1, int x2, int y2, int &ans){
	
	//local variables
	int q1, s1, q2, s2;

	calc_obj_quadrant_sector(x1, y1, q1, s1);
	calc_obj_quadrant_sector(x2, y2, q2, s2);

	if (q1 == 1 && q2 == 3) ans = 1;
	else if (q1 == 2 && q2 == 4) ans = 1;
	else if (q1 == 3 && q2 == 1) ans = 1;
	else if (q1 == 4 && q2 == 1) ans = 1;
	else ans = 0;


	return 0;
}

//fn to calculate the distance b/w two points
int calc_dist(int x1, int y1, int x2, int y2, double &dist){

	dist = sqrt((double)(x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));

	return 0;
}

//fn used to calcualte the best safe location, given two options (based on distance required to travel)
int calc_best_option(double opx1, double opy1, double opth1, double sqf1, int side1, double opx2, double opy2, double opth2, double sqf2, int side2, double &xgoal, double &ygoal, double &phi_goal, double &sqf, int &side, int x_robot, int y_robot, int x_opp, int y_opp){
	
	//local variables
	double distrobot1, distrobot2;
	double distop1, distop2;
	
	calc_dist(x_robot, y_robot, (int)opx1, (int)opy1, distrobot1);
	calc_dist(x_robot, y_robot, (int)opx2, (int)opy2, distrobot2);
	
	//1-- look for closest obstacle to our robot
	if (distrobot1 < distrobot2){ xgoal = opx1;	ygoal = opy1;	phi_goal = opth1; sqf = sqf1; side = side1; }
	else if (distrobot1 > distrobot2){ xgoal = opx2;	ygoal = opy2;	phi_goal = opth2; sqf = sqf2; side = side2; }
	else{
		calc_dist(x_opp, y_opp, (int)opx1, (int)opy1, distop1);
		calc_dist(x_opp, y_opp, (int)opx2, (int)opy2, distop2);
		//2-- check for furthest obstacle from enemy robot
		if (distop1 < distop2){ xgoal = opx2;	ygoal = opy2;	phi_goal = opth2; sqf = sqf2; side = side2; }
		else if (distop1 > distop2){ xgoal = opx1;	ygoal = opy1;	phi_goal = opth1; sqf = sqf1; side = side1; }
		else{ xgoal = opx1;	ygoal = opy1;	phi_goal = opth1; sqf = sqf1; side = side1; }
	}

	return 0;
}

//fn used to check if two points are on opposite ends <-- final vers (UNUSED)
int calc_check_opposing_side(int &check, int xo1, int yo1, int xo2, int yo2, int xminb, int xmaxb, int yminb, int ymaxb){
	
	if (xo1 <= xminb && xo2 >= xmaxb)		check = 1;
	else if (xo1 >= xmaxb && xo2 <= xminb)	check = 1;
	else if (yo1 <= yminb && yo2 >= ymaxb)	check = 1;
	else if (yo1 >= ymaxb && yo2 <= yminb)	check = 1;
	else									check = 0;

	return 0;
}

//fn used to check if two points have a common side	 <-- final vers (UNUSED)
int calc_check_common_side(int &sidecheck, int &side, int xo1, int yo1, int xo2, int yo2, int xminb, int xmaxb, int yminb, int ymaxb){

	if (xo1 <= xminb && xo2 <= xminb) {			sidecheck = 1;	side = 1; }
	else if (xo1 >= xmaxb && xo2 >= xmaxb) {	sidecheck = 1;  side = 3; }
	else if (yo1 <= yminb && yo2 <= yminb) {	sidecheck = 1;  side = 4; }
	else if (yo1 >= ymaxb && yo2 >= ymaxb) {	sidecheck = 1;  side = 2; }
	else {										sidecheck = 0;  side = 0; }

	return 0;
}

//fn used to calculate the theta for two obstacles	<-- final vers (UNUSED)
int calc_phi_obs(double &phiobs, int xobs1, int yobs1, int xobs2, int yobs2){

	//local variables
	double f = 180.0 / 3.14159;

	if (xobs1 < xobs2){			phiobs = atan2((yobs2 - yobs1), (xobs2 - xobs1))*(f);	}
	else if (xobs1>xobs2){		phiobs = atan2((yobs1 - yobs2), (xobs1 - xobs2))*(f);	}
	else if (xobs1 == xobs2){
		if (yobs1>yobs2){		phiobs = atan2((yobs1 - yobs2), (xobs1 - xobs2))*(f);		}
		else if (yobs1<yobs2){	phiobs = atan2((yobs2 - yobs1), (xobs2 - xobs1))*(f);	}
	}

	return 0;
}

//calculate bounds of obstacle , given centroid 
int calc_obs_outer_box(int &xminouter, int &xmaxouter, int &yminouter, int &ymaxouter, int xobs1, int yobs1, int xobs2, int yobs2, int d_obs){

	//local variables
	int b_value = (int)round(d_obs / 2.0);

	if (xobs1 <= xobs2)	{		xminouter = xobs1 - b_value;	xmaxouter = xobs2 + b_value; }
	else if (xobs1 > xobs2) {	xminouter = xobs2 - b_value;	xmaxouter = xobs1 + b_value; }
	
	if (yobs1 <= yobs2)	{		yminouter = yobs1 - b_value;	ymaxouter = yobs2 + b_value; }
	else if (yobs1 > yobs2) {	yminouter = yobs2 - b_value;	ymaxouter = yobs1 + b_value; }


	return 0;
}

//calculate inner bounds of two obstacles, given centroids
int calc_obs_inner_box(int &xmininner, int &xmaxinner, int &ymininner, int &ymaxinner, int xobs1, int yobs1, int xobs2, int yobs2, int d_obs){

	//local variables
	int b_value = (int)round(d_obs / 2.0);

	if (xobs1 <= xobs2)	{		xmininner = xobs1 + b_value;	xmaxinner = xobs2 - b_value; }
	else if (xobs1 > xobs2) {	xmininner = xobs2 + b_value;	xmaxinner = xobs1 - b_value; }

	if (yobs1 <= yobs2)	{		ymininner = yobs1 + b_value;	ymaxinner = yobs2 - b_value; }
	else if (yobs1 > yobs2) {	ymininner = yobs2 + b_value;	ymaxinner = yobs1 - b_value; }

	return 0;

}

//fn used to order 3 values, given three squish factors in inc order smallest to largest)
//created mainly to reduce function bulk
int calc_order_park_inc(int *park, int p1, int p2, int p3){

	if (p1<p2){
		if (p1<p3){
			if (p2 < p3) {		park[1] = p1; park[2] = p2; park[3] = p3; }
			else if (p3<p2){	park[1] = p1; park[2] = p3; park[3] = p2; }
			else if (p2 == p3){ park[1] = p1; park[2] = p2; park[3] = p3; }
		}
		else if (p3<p1){		park[1] = p3; park[2] = p1; park[3] = p2; }
		else if (p1 == p3){		park[1] = p1; park[2] = p3; park[3] = p2; }
	}
	else if (p2<p1){
		if (p2<p3){
			if (p3 < p1){		park[1] = p2; park[2] = p3; park[3] = p1; }
			else if (p3 > p1){	park[1] = p2; park[2] = p1; park[3] = p3; }
			else if (p3 == p1){ park[1] = p2; park[2] = p1; park[3] = p3; }
		}
		else if (p3 < p2){		park[1] = p3; park[2] = p2; park[3] = p1; }
		else if (p2 == p3){		park[1] = p2; park[2] = p3; park[3] = p1; }
	}
	else if (p1 == p2){
		if (p3<p1){				park[1] = p3; park[2] = p1; park[3] = p2; }
		else if (p3>p1){		park[1] = p1; park[2] = p2; park[3] = p3; }
		else if (p3 == p1){		park[1] = p1; park[2] = p2; park[3] = p3; }
	}


	return 0;
}



