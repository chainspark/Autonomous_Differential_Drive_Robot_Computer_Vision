////
//FINAL VERSION <-- DeathStar (h)
//TEAM DeathStar (MECH 6631 - Robot Light Wars)
//		||Members
//		||Chandan Satija
//		||Tasnim Haque
//		||Farshad Zaboli
//		||Mohammad Saeidi


//Input Fn
int initialize_c_array(int *c1, int *c2, int *c3, int *c4);

int populate_c_array(int *c, int r, int g, int b);


// RGB Image Convolution Filtering

int rgb_convolution(image &a, image &b, int *k, double s);

int rgb_lowpass_filter(image &a, image &b);

int rgb_highpass_filter(image &a, image &b);

int rgb_gaussian_filter(image &a, image &b);


// Mask Functions
int sobel(image &a, image &mag, image &theta);

int create_mask(int mask_ci, int mask_cj, int mask_height, int mask_width, image &mask, image &b, double mask_angle);

int mask_visualizer(image &mask, image &rgb, int mask_ci, int mask_cj, double mask_angle);

int set_mask(int pi, int pj, int mask_height, double phi, int &mask_ci, int &mask_cj, double & mask_angle);

int tf_workspace_to_mask(int x_obs, int y_obs, int mask_ci, int mask_cj, int &mi_obs, int &mj_obs, double mask_angle);


// Centroid Tracking and Label Information Filtering

//	Object Filtering / Identification

int size_label(image &label, int nlabel, int *x_size1, int *x_size2, int *y_size1, int *y_size2);

int rgb_2_hsv(int red, int green, int blue, int &hue, int &sat, int &val);

int initialize_robot_arrays(int *r1, int *r2, int size);

int initialize_obstacle_arrays(int *obs, int *obs_s, int size);

int update_obstacle_size(int *obs, int *obs_s, int size, int dim, int pos);

int rgb_location(image &a, double ic, double jc, int &rc, int &gc, int &bc);

int hsv_location(image &a, double ic, double jc, int &hc, int &sc, int &vc);

int same_hue(int c1, int c2);

int combine_label(int *object_array, int size, int label, int hsv, double ic, double jc, int pos);

int filter_objects(int &nlabels, image &label, int *x_size1, int *x_size2, int *y_size1, int *y_size2, image &b, image &rgb0, int *c1, int *c2, int *c3, int *c4, int *robot_a, int *robot_b, int *obstacles, int *obstacles_size, int &size_r, int &size_o);

int filter_objects_update(int &nlabels, image &label, int *x_size1, int *x_size2, int *y_size1, int *y_size2, image &b, image &rgb0, int *c1, int *c2, int *c3, int *c4, int *robot_a, int *robot_b, int *obstacles, int *obstacles_size, int &size_r, int &size_o);

int filter_objects_check(int &x_robot, int &y_robot, int &d_SF, int &nlabels_actual, int &x_opp, int &y_opp, int *x_obs_array, int *y_obs_array, int &nlabels, image &label, int *x_size1, int *x_size2, int *y_size1, int *y_size2, image &b, image &rgb0, int *c1, int *c2, int *c3, int *c4, int *robot_a, int *robot_b, int *obstacles, int *obstacles_size, int &size_r, int &size_o);

int update_label(int *object_array, int size, int label, int hsv, double ic, double jc);

int check_label(int	*object_array, int size, int label);

//	Object Tracking

int search_object(i2byte &nlabel, image &label, int is, int js);

int track_object(int *r_array, int size, image &label, image &b, image &rgb0);


//Control Functions

int align_full_throttle(double phi_robot, double phi_goal, int &pw_l, int &pw_r);

int align_closed_loop(double phi_robot, double phi_goal, double dt, int &pw_l, int &pw_r,double &ep,double &ei);

int move_forward(int &pw_l, int &pw_r, double &ep, double &ei);

int move_backward(int &pw_l, int &pw_r, double &ep, double &ei);

int align_lock(double phi_robot, double phi_goal, double dt, int &pw_l, int &pw_r, double &ep, double &ei, double tolerance);

int align_fast(double phi_robot, double phi_goal, int &pw_l, int &pw_r, double tolerance);

int stop_robot(int &pw_l, int &pw_r);