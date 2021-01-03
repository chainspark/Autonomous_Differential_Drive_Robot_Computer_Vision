////
//FINAL VERSION <-- Defend (h)
//TEAM DeathStar (MECH 6631 - Robot Light Wars)
//		||Members
//		||Chandan Satija
//		||Tasnim Haque
//		||Farshad Zaboli
//		||Mohammad Saeidi


//defend functions

//main function
int defend(int obs_number, int *x_obs_array, int *y_obs_array, int *obs_size, double &xgoal, double &ygoal, double &phi_goal, int x_robot, int y_robot, double phi_robot, int x_opp, int y_opp, double phi_opp);


//scenario 1 - 1 centered obstacle					--> circling
int defend_s1(double &xgoal, double &ygoal, double &phi_goal, int &x_obs_near, int &y_obs_near, int &x_opp, int &y_opp, double &phi_opp, int &d_obs, int &d_SF);

//scenario 2 - 2 centered obstacles					--> circling
int defend_s2(double &xgoal, double &ygoal, double &phi_goal, int &x_obs_b_1, int &y_obs_b_1, int &x_obs_b_2, int &y_obs_b_2, int &x_opp, int &y_opp, double &phi_opp, int &d_obs, int &d_SF);

//scenario 3 - 1 bordering obstacle					--> park along edge, facing out
int defend_s3(double &xgoal, double &ygoal, double &phi_goal, double &sqf, int &side, int &x_obs, int &y_obs, int &x_opp, int &y_opp, double &phi_opp, int &x_robot, int &y_robot, double &phi_robot, int &d_obs, int &d_SF, int &d_rw, int &d_rl);

//scenario 4 - 2 bordering obstacles				--> park along edge, facing out
int defend_s4(double &xgoal, double &ygoal, double &phi_goal, int &x_obs_ub_1, int &y_obs_ub_1, int &x_obs_ub_2, int &y_obs_ub_2, int &x_opp, int &y_opp, double &phi_opp, int &x_robot, int &y_robot, double &phi_robot, int &d_obs, int &d_SF, int &d_rw, int &d_rl);

//scenario 5 - 1 centered, 1 borerdering obstacle	--> if circling possible, circle else park
int defend_s5(double &xgoal, double &ygoal, double &phi_goal, int &x_obs_b_1, int &y_obs_b_1, int &x_obs_ub_2, int &y_obs_ub_2, int &x_opp, int &y_opp, double &phi_opp, int &x_robot, int &y_robot, double &phi_robot, int &d_obs, int &d_SF, int &l_robot, int &w_robot);




//helper functions
int calc_obs_bounded(int &obs_number_b, int &obs_number, int *x_obs_array, int *y_obs_array, int *obs_b, int *x_obs_b, int *y_obs_b, int x_b_min, int x_b_max, int y_b_min, int y_b_max);

int calc_obj_orientation_2_xy(int x_opp, int y_opp, double phi_opp, int x_obs, int y_obs, double &inc_angle);

int calc_obj_quadrant_sector(int x, int y, int &q, int &s);

int check_quadrant_opp(int x1, int y1, int x2, int y2, int &ans);

int calc_dist(int x1, int y1, int x2, int y2, double &dist);

int calc_best_option(double opx1, double opy1, double opth1, double sqf1, int side1, double opx2, double opy2, double opth2, double sqf2, int side2, double &xgoal, double &ygoal, double &phi_goal, double &sqf, int &side, int x_robot, int y_robot, int x_opp, int y_opp);

int calc_check_opposing_side(int &check, int xo1, int yo1, int xo2, int yo2, int xminb, int xmaxb, int yminb, int ymaxb);

int calc_check_common_side(int &sidecheck, int &side, int xo1, int yo1, int xo2, int yo2, int xminb, int xmaxb, int yminb, int ymaxb);

int calc_phi_obs(double &phiobs, int xobs1, int yobs1, int xobs2, int yobs2);

int calc_obs_outer_box(int &xminouter, int &xmaxouter, int &yminouter, int &ymaxouter, int xobs1, int yobs1, int xobs2, int yobs2, int d_obs);

int calc_obs_inner_box(int &xmininner, int &xmaxinner, int &ymininner, int &ymaxinner, int xobs1, int yobs1, int xobs2, int yobs2, int d_obs);

int calc_order_park_inc(int *park, int p1, int p2, int p3);