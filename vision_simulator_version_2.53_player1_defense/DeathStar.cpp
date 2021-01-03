////
//FINAL VERSION <-- DeathStar (cpp)
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

#include "Defend.h"


//Input Fn / Colour Initialization for robots 
int initialize_c_array(int *c1, int *c2, int *c3, int *c4){
	//purpose: to initialise colour arrays for each robot

	//TODO: take input from keyboard for robota colours
	int c1_R = 73, c1_G = 181, c1_B = 134;		//colour1/colour1-our robot
	int c2_R = 221, c2_G = 91, c2_B = 78;		//colour2/colour2-our robot
	populate_c_array(c1, c1_R, c1_G, c1_B);
	populate_c_array(c2, c2_R, c2_G, c2_B);


	//TODO: take input from keyboard for robotb/opp colours
	int c3_R = 255, c3_G = 189, c3_B = 124;		//colour3/colour1-opp robot
	int c4_R = 50, c4_G = 160, c4_B = 230;		//colour4/colour2-opp robot
	populate_c_array(c3, c3_R, c3_G, c3_B);
	populate_c_array(c4, c4_R, c4_G, c4_B);

	return 0;
}

int populate_c_array(int *c, int r, int g, int b){
	//purpose: to convert [rgb] values, provided at beginning of program, to [hsv]
	
	//local variables
	int h, s, v;

	c[1] = r;	c[2] = g;	c[3] = b;
	rgb_2_hsv(c[1], c[2], c[3], h, s, v);
	c[4] = h;	c[5] = s;	c[6] = v;

	return 0;
}


// RGB Image convolution Filtering
int rgb_convolution(image &a, image &b, int *k, double s)
// perform a 3 X 3 convolution filter with the following kernel
// k1 k2 k3
// k4 k5 k6
// k7 k8 k9
{
	i4byte size, i, j;
	ibyte *pa, *pa1, *pa2, *pa3, *pa4, *pa5, *pa6, *pa7, *pa8, *pa9, *pb;
	i2byte width, height;
	int x;

	// check for compatibility of a, b
	if (a.height != b.height || a.width != b.width) {
		printf("\nerror in convolution: sizes of a, b are not the same!");
		return 1;
	}

	if (a.type != RGB_IMAGE || b.type != RGB_IMAGE) {
		printf("\nerror in convolution: input types are not valid!");
		return 1;
	}

	width = a.width;
	height = a.height;

	// initialize pointers
	pa = a.pdata + 3*width + 3*1;
	pb = b.pdata + 3*width + 3*1;

	// set neighbourhood pointers
	// make sure they don't point outside of the images at the boundaries
	// when you use them
	// pa7 pa8 pa9
	// pa4 pa5 pa6
	// pa1 pa2 pa3

	pa1 = pa - 3*width - 3*1;
	pa2 = pa - 3*width;
	pa3 = pa - 3*width + 3*1;
	pa4 = pa - 3*1;
	pa5 = pa;
	pa6 = pa + 3*1;
	pa7 = pa + 3*width - 3*1;
	pa8 = pa + 3*width;
	pa9 = pa + 3*width + 3*1;

	// number of pixels to process
	size = (i4byte)a.width * a.height - 2 * width - 2;

	// perform 3x3 convolution filter
	for (i = 0; i<3*size; i++) {
		// set the center pixel equal to the weighted sum of the pixels
		x = k[1] * (*pa1) + k[2] * (*pa2) + k[3] * (*pa3) +
			k[4] * (*pa4) + k[5] * (*pa5) + k[6] * (*pa6) +
			k[7] * (*pa7) + k[8] * (*pa8) + k[9] * (*pa9);

		x = (int)(s*x); // apply the scale factor

		// check for out of range values
		if (x < 0)   x = 0;
		if (x > 255) x = 255;

		*pb = (ibyte)x;

		// increment pointers -> move the neighbourhood right
		pa1++; pa2++; pa3++; pa4++; pa5++;
		pa6++; pa7++; pa8++; pa9++; pb++;
	}

	// initialize pointers
	pa = a.pdata;
	pb = b.pdata;

	// number of pixels
	size = (i4byte)a.width * a.height;

	// process the image borders
	for (i = 0; i<=(width-1)*3+2; i++) {
		pb[i] = pb[i + width*3]; // bottom
		pb[size*3 - i -3*1] = pb[size*3 - i -3*1 -width*3]; // top
	}
	
	for (i = 0, j = 0; i<height; i++, j += width*3) {
		pb[j] = pb[j + 1*3];
		pb[j+1] = pb[j+1 + 1*3];
		pb[j+2] = pb[j+2 + 1*3]; // right
		pb[size*3 - j - 1*3] = pb[size*3 - j - 2*3];
		pb[size*3 - j+1 - 1*3] = pb[size*3 - j+1 - 2*3];
		pb[size*3 - j+2 - 1*3] = pb[size*3 - j+2 - 2*3]; // left
	}
	
	
	pb[0] = pb[width*3 + 1*3];// bottom left corner
	pb[0+1] = pb[width*3 + 1*3+1];
	pb[0+2] = pb[width*3 + 1*3+2];
	pb[width*3 - 1*3] = pb[width*3 - 1*3 + width*3 - 1*3]; // top right corner
	pb[width * 3 - 1 * 3 +1] = pb[width * 3 - 1 * 3 + width * 3 - 1 * 3 +1];
	pb[width * 3 - 1 * 3 +2] = pb[width * 3 - 1 * 3 + width * 3 - 1 * 3 + 2] ;
	pb[size*3 - width*3] = pb[size*3 - width*3 - width*3 + 1*3]; // bottom left corner
	pb[size * 3 - width * 3 +1] = pb[size * 3 - width * 3 - width * 3 + 1 * 3+1];
	pb[size * 3 - width * 3 +2] = pb[size * 3 - width * 3 - width * 3 + 1 * 3+2];
	pb[size*3 - 1*3] = pb[size*3 - 1*3 - width*3 - 1*3]; // bottom right corner
	pb[size * 3 - 1 * 3+1] = pb[size * 3 - 1 * 3 - width * 3 - 1 * 3+1];
	pb[size * 3 - 1 * 3+2] = pb[size * 3 - 1 * 3 - width * 3 - 1 * 3+2];
	
	return 0;
}

 int rgb_lowpass_filter(image &a, image &b)
{
	int k[10];
	double s;

	// define convolution kernel
	// k1 k2 k3
	// k4 k5 k6
	// k7 k8 k9

	k[1] = 1;
	k[2] = 1;
	k[3] = 1;
	k[4] = 1;
	k[5] = 1;
	k[6] = 1;
	k[7] = 1;
	k[8] = 1;
	k[9] = 1;

	s = 1.0 / 9;

	rgb_convolution(a, b, k, s);

	return 0;
}

 int rgb_highpass_filter(image &a, image &b)
 {
	 int k[10];
	 double s;

	 // define convolution kernel
	 // k1 k2 k3
	 // k4 k5 k6
	 // k7 k8 k9

	 k[1] = -1;
	 k[2] = -1;
	 k[3] = -1;
	 k[4] = -1;
	 k[5] = 9;
	 k[6] = -1;
	 k[7] = -1;
	 k[8] = -1;
	 k[9] = -1;

	 s = 1.0;

	 rgb_convolution(a, b, k, s);

	 return 0;
 }

 int rgb_gaussian_filter(image &a, image &b)
 {
	 int k[10];
	 double s;

	 // define convolution kernel
	 // k1 k2 k3
	 // k4 k5 k6
	 // k7 k8 k9

	 k[1] = 1;
	 k[2] = 2;
	 k[3] = 1;
	 k[4] = 2;
	 k[5] = 4;
	 k[6] = 2;
	 k[7] = 1;
	 k[8] = 2;
	 k[9] = 1;

	 s = 1.0 / 16;

	 rgb_convolution(a, b, k, s);

	 return 0;
 }


// Mask Functions

 int sobel(image &a, image &mag, image &theta)
 {
	 i4byte size, i, j;
	 ibyte *pa, *pa1, *pa2, *pa3, *pa4, *pa5, *pa6, *pa7, *pa8, *pa9;
	 ibyte *p_mag, *p_theta;
	 i2byte width, height;

	 // note we use a signed in here since sx, sy could be < 0
	 int sx, sy, M;
	 int kx[10], ky[10];
	 double A;

	 // check for compatibility image sizes and types
	 if (a.height != mag.height || a.width != mag.width ||
		 a.height != theta.height || a.width != theta.width)
	 {
		 printf("\nerror in convolution: sizes images are not the same!");
		 return 1;
	 }

	 if (a.type != GREY_IMAGE || mag.type != GREY_IMAGE
		 || theta.type != GREY_IMAGE)
	 {
		 printf("\nerror in convolution: input types are not valid!");
		 return 1;
	 }

	 width = a.width;
	 height = a.height;

	 // initialize pointers
	 pa = a.pdata + width + 1;
	 p_mag = mag.pdata + width + 1;
	 p_theta = theta.pdata + width + 1;

	 // set neighbourhood pointers

	 // make sure they don't point outside of the images at the boundaries
	 // when you use them

	 // note the order of the neighbourhood is correctly given below
	 // as discussed in class (the old order was for a different
	 // image coord system in an older version of the library).
	 // pa7 pa8 pa9
	 // pa4 pa5 pa6
	 // pa1 pa2 pa3
	 pa1 = pa - width - 1;
	 pa2 = pa - width;
	 pa3 = pa - width + 1;
	 pa4 = pa - 1;
	 pa5 = pa;
	 pa6 = pa + 1;
	 pa7 = pa + width - 1;
	 pa8 = pa + width;
	 pa9 = pa + width + 1;

	 // number of pixels to process
	 size = (i4byte)a.width * a.height - 2 * width - 2;

	 // set convolution coefficients for sx and sy
	 // k7 k8 k9
	 // k4 k5 k6
	 // k1 k2 k3
	 kx[7] = -1; kx[8] = 0; kx[9] = 1;
	 kx[4] = -2; kx[5] = 0; kx[6] = 2;
	 kx[1] = -1; kx[2] = 0; kx[3] = 1;

	 ky[7] = 1;  ky[8] = 2;  ky[9] = 1;
	 ky[4] = 0;  ky[5] = 0;  ky[6] = 0;
	 ky[1] = -1; ky[2] = -2; ky[3] = -1;

	 // calculate sx and sy
	 // here I calculate both at the same time in the loop
	 // since I don't want to store them into an image array
	 // (they can't store negative numbers which might occur for sx
	 // and sy) and I need both to calculate mag and theta.
	 for (i = 0; i<size; i++) {

		 sx = kx[1] * (*pa1) + kx[2] * (*pa2) + kx[3] * (*pa3) +
			 kx[4] * (*pa4) + kx[5] * (*pa5) + kx[6] * (*pa6) +
			 kx[7] * (*pa7) + kx[8] * (*pa8) + kx[9] * (*pa9);

		 sy = ky[1] * (*pa1) + ky[2] * (*pa2) + ky[3] * (*pa3) +
			 ky[4] * (*pa4) + ky[5] * (*pa5) + ky[6] * (*pa6) +
			 ky[7] * (*pa7) + ky[8] * (*pa8) + ky[9] * (*pa9);

		 // might consider directly substituting kx, ky above
		 // to reduce computation time

		 // calculate mag and theta
		 M = abs(sx) + abs(sy); // fast approx of sqrt(sx*sx + sy*sy)

		 if (M > 255) M = 255; // check for overflow
		 // alternatively M can be scaled by 1/2, 1/4, 1/8
		 // to reduce (1/2) or avoid (1/8) possibility of overlow

		 *p_mag = M;

		 A = atan2((double)sy, (double)sx) / 3.14159 * 180; // deg
		 // note that A ranges from -180 to 180 deg

		 // scale A so that it ranges from 0 to 255
		 // and will fit in a greyscale image range
		 // -- add 0.01 to account for roundoff error
		 A = (A + 180) / 360 * 255 + 0.01;

		 *p_theta = (int)A;

		 // note this line might be useful to cut down
		 // on the noise / irrelevant info from theta
		 if (M < 75) *p_theta = 0;

		 // increment pointers
		 pa1++; pa2++; pa3++; pa4++; pa5++;
		 pa6++; pa7++; pa8++; pa9++;
		 p_mag++, p_theta++;
	 }

	 // copy edges of image from valid regions
	 p_mag = mag.pdata;
	 p_theta = theta.pdata;

	 // number of pixels
	 size = (i4byte)a.width * a.height;

	 for (i = 0; i<width; i++) {
		 p_mag[i] = p_mag[i + width]; // bottom
		 p_mag[size - i - 1] = p_mag[size - i - 1 - width]; // top
		 p_theta[i] = p_theta[i + width]; // bottom
		 p_theta[size - i - 1] = p_theta[size - i - 1 - width]; // top
	 }

	 for (i = 0, j = 0; i<height; i++, j += width) {
		 p_mag[j] = p_mag[j + 1]; // left
		 p_mag[size - j - 1] = p_mag[size - j - 2]; // right
		 p_theta[j] = p_theta[j + 1]; // left
		 p_theta[size - j - 1] = p_theta[size - j - 2]; // right
	 }

	 return 0;
 }

 int create_mask(int mask_ci, int mask_cj, int mask_height, int mask_width, image &mask, image &b, double mask_angle)
	 // mask_ci - X coordinates of Mask origin
	 // mask_cj - Y coordinates of Mask origin
	 // b - Grey Image on which mask is applied
	 // mask - Grey Mask Image for further processing

 {
	 int i, j;
	 int width, height;
	 int imask, jmask, mask_pixel;

	 ibyte *pb, *pmask;

	 width = b.width;
	 height = b.height;

	 for (j = 0; j <= mask_height - 1; j++){
		 for (i = 0; i <= mask_width - 1; i++){

			 //Mask pixel co-ordinates in the reference image

			 imask = mask_ci + cos(((mask_angle) / 180.00)*3.14159)*(double)i - sin(((mask_angle) / 180.00)*3.14159)*(double)j;
			 jmask = mask_cj + sin(((mask_angle) / 180.00)*3.14159)*(double)i + cos(((mask_angle) / 180.00)*3.14159)*(double)j;

			 // Limit from going off the screen
			 if (imask < 0) imask = 0;
			 if (imask > width - 1) imask = width - 1;
			 if (jmask < 0) jmask = 0;
			 if (jmask > height - 1) jmask = height - 1;

			 // Mask pixel cordinates in the reference image
			 mask_pixel = (int)(width*jmask + imask);

			 pb = b.pdata;
			 // Copy pixel to mask image for processing
			 pmask = mask.pdata;
			
			 pmask[(mask_width)*(j)+(i)] = pb[mask_pixel];
		 }
	 }
	 return 0;
 }

 int mask_visualizer(image &mask, image &rgb, int mask_ci, int mask_cj, double mask_angle)
	 // Mask image - Grey Image
	 // RGB image on which mask is imposed for visualisation
 {
	 int i, j;
	 int width, height;
	 int imask, jmask, mask_pixel;

	 int mask_width, mask_height;

	 mask_width = mask.width;
	 mask_height = mask.height;

	 ibyte *prgb, *pmask;

	 width = rgb.width;
	 height = rgb.height;

	 prgb = rgb.pdata;
	 pmask = mask.pdata;

	 for (i = 0; i < mask_width; i++){
		 for (j = 0; j < mask_height; j++){

			 //Mask pixel co-ordinates in the reference image- Use this to tranfer information from mask image to world frame

			 imask = mask_ci + cos(((mask_angle) / 180.00)*3.14159)*(double)i - sin(((mask_angle) / 180.00)*3.14159)*(double)j;
			 jmask = mask_cj + sin(((mask_angle) / 180.00)*3.14159)*(double)i + cos(((mask_angle) / 180.00)*3.14159)*(double)j;

			 // Limit from going off the screen
			 if (imask < 0) imask = 0;
			 if (imask > width - 1) imask = width - 1;
			 if (jmask < 0) jmask = 0;
			 if (jmask > height - 1) jmask = height - 1;

			 mask_pixel = (int)(width*jmask + imask);

			 prgb[3 * mask_pixel] = pmask[(mask_width)*(j)+(i)];
			 prgb[3 * mask_pixel + 1] = pmask[(mask_width)*(j)+(i)];
			 prgb[3 * mask_pixel + 2] = pmask[(mask_width)*(j)+(i)];
		 }
	 }
	 return 0;
 }

 int set_mask(int pi, int pj, int mask_height, double phi, int &mask_ci, int &mask_cj, double & mask_angle)
 {
	 if (phi < 45.00 && phi >= -45.00)
	 {
		 mask_ci = pi;
		 mask_cj = pj - mask_height / 2;
		 mask_angle = 0;
	 }
	 else if (phi < 135.00 && phi >= 45.00)
	 {
		 mask_ci = pi + mask_height / 2;
		 mask_cj = pj;
		 mask_angle = 90.00;
	 }
	 else if ((phi <= 180.00 && phi >= 135.00) || (phi > -180.00 && phi < -135.00))
	 {
		 mask_ci = pi;
		 mask_cj = pj + mask_height / 2;
		 mask_angle = 180.00;
	 }
	 else if (phi < -45.00 && phi >= -135.00)
	 {
		 mask_ci = pi - mask_height / 2;
		 mask_cj = pj;
		 mask_angle = 270.00;
	 }
	 return 0;
 }

 int tf_workspace_to_mask(int x_obs, int y_obs,int mask_ci,int mask_cj,int &mi_obs,int &mj_obs,double mask_angle)
 {
	 // x_obs,y_obs - Obstacles co-ordinates in world frame
	 // mi_obs,mj_obs - Obstacles co-ordinates in mask frame
	 // Control Logic will taken locally w.r.t mask frame
	 mi_obs = cos(((-mask_angle) / 180.00)*3.14159)*(x_obs - mask_ci) - sin(((-mask_angle) / 180.00)*3.14159)*(y_obs - mask_cj);
	 mj_obs = sin(((-mask_angle) / 180.00)*3.14159)*(x_obs - mask_ci) + cos(((-mask_angle) / 180.00)*3.14159)*(y_obs - mask_cj);
	 return 0;

 }


 // Centroid Tracking and Label Information Filtering
 
 // Object Filtering/Identification 

 int rgb_2_hsv(int red, int green, int blue, int &hue, int &sat, int &val){
	 //purpose: to convert an [rgb] value to an [hsv] value

	 //local variables
	 double r, g, b;
	 double h, s, v;
	 double cmax, cmin, delta;
	 double min = 1, max = 0;

	 r = red / 255.0;
	 g = green / 255.0;
	 b = blue / 255.0;

	 int i = 0;
	 for (i = 0; i < 3; i++){
		 if (max < r) max = r;
		 if (max < g) max = g;
		 if (max < b) max = b;

		 if (min > r) min = r;
		 if (min > g) min = g;
		 if (min > b) min = b;
	 }

	 cmax = max;
	 cmin = min;
	 delta = cmax - cmin;

	 //hue
	 if (delta == 0) h = 0.0;
	 else if (cmax == r) h = 60.0*(fmod(((g - b) / delta), 6.0));
	 else if (cmax == g) h = 60.0*(((b - r) / delta) + 2.0);
	 else if (cmax == b) h = 60.0*(((r - g) / delta) + 4.0);

	 //sat
	 if (cmax == 0) s = 0.0;
	 else if (cmax != 0) s = (delta / cmax);

	 //value
	 v = cmax;

	 hue = (int)round(h);
	 sat = (int)round(s*100.0);		
	 val = (int)round(v*100.0);

	 return 0;

 }

 int size_label(image &label, int nlabel, int *x_size1, int *x_size2, int *y_size1, int *y_size2)

 {

	 int i, j;

	 int width, height;

	 i2byte *pl;

	 pl = (i2byte *)label.pdata;


	 // number of pixels

	 width = label.width;

	 height = label.height;

	 //Size of label object is Size2-Size1 in the respective direction

	 x_size1[nlabel] = width + 1;

	 x_size2[nlabel] = -1;

	 y_size1[nlabel] = height + 1;

	 y_size2[nlabel] = -1;



	 int k = nlabel;


	 for (j = 0; j < height; j++) { // y-dir

		 for (i = 0; i < width; i++) { // x-dir

			 if (pl[j*width + i] == nlabel) {

				 if (x_size1[nlabel] > i) x_size1[nlabel] = i;

				 if (x_size2[nlabel] < i) x_size2[nlabel] = i;

				 if (y_size1[nlabel] > j) y_size1[nlabel] = j;

				 if (y_size2[nlabel] < j) y_size2[nlabel] = j;

			 }

		 }

	 }

	 return 0;

 }

 // Centroid Tracking Functions
 int initialize_robot_arrays(int *r1, int *r2, int size){
	 //purpose: to initialise both robot arrays robot_a/our robot & robot_b/enemy robot

	 //local variables
	 int i;
	 
	 for (i = 1; i <= (size)* 4 + 1; i++){
		 r1[i] = 0;
		 r2[i] = 0;
	 }

	 return 0;
 }

 int initialize_obstacle_arrays(int *obs, int*obs_s, int size){
	 //purpose: to initialize obstacle related arrays (obstacles, obstacles_size)

	 //local variables
	 int i, n=0;
	 
	 for (i = 1; i <= (size)* 4 + 1; i++){	obs[i] = 0;	}			//initialise the obstacles array (holds value, label, i, j, for each obstacle)
	 for (i = 1; i <= size; i++){	obs_s[i] = 0;	}				//initialise obstacles_size array (holds dimensions)
	 
	 //in the case where only one obstacle is found on the court
	 if (size == 1) obs_s[2] = -1;
	 

	 return 0;
 }

 int update_obstacle_size(int *obs, int *obs_s, int size, int dim, int pos){
	 //purpose: to store obstacle size info

	 //local variables
	 int i;
	 int n1, n21, n22;
	 int s, s1, s2;

	 obs_s[pos] = dim;

	 return 0;
 }

 int rgb_location(image &a, double ic, double jc, int &rc, int &gc, int &bc){
	 //purpose: to retrieve rgb value at specified location i,j

	 //local variables
	 ibyte *pa; // R, G, B;
	 i4byte w, h; //i, j, k, 
	 int kc;

	 if (a.type != RGB_IMAGE){
		 printf("\nerror in centroid: input types are not valid!");
		 return 1;
	 }

	 pa = a.pdata;
	 w = a.width;
	 h = a.height;
	 kc = ((int)jc)*w + (int)ic;

	 bc = pa[3 * kc];
	 gc = pa[3 * kc + 1];
	 rc = pa[3 * kc + 2];


	 return 0;
 }

 int hsv_location(image &a, double ic, double jc, int &hc, int &sc, int &vc){
	 //purpose: to retrieve hsv value at specified i,j location

	 //local variables
	 ibyte *pa; // R, G, B;
	 i4byte w, h; //i, j, k, 
	 int kc;

	 int rc, gc, bc;

	 if (a.type != RGB_IMAGE){
		 printf("\nerror in centroid: input types are not valid!");
		 return 1;
	 }

	 pa = a.pdata;
	 w = a.width;
	 h = a.height;
	 kc = ((int)jc)*w + (int)ic;

	 bc = pa[3 * kc];
	 gc = pa[3 * kc + 1];
	 rc = pa[3 * kc + 2];

	 rgb_2_hsv(rc, gc, bc, hc, sc, vc);


	 return 0;
 }

 int same_hue(int c1, int c2){
	 //purpose: to check whether two colours c1,c2 provided are of the same hue

	 if ((c1 >= 0) && (c1 < 30) && (c2 >= 0) && (c2 < 30)) return 1;				//red			
	 else if ((c1 >= 30) && (c1 < 60) && (c2 >= 30) && (c2 < 60)) return 1;			//orange
	 else if ((c1 >= 60) && (c1 < 90) && (c2 >= 60) && (c2 < 90)) return 1;			//yellow
	 else if ((c1 >= 90) && (c1 < 180) && (c2 >= 90) && (c2 < 180)) return 1;		//green
	 else if ((c1 >= 180) && (c1 < 210) && (c2 >= 180) && (c2 < 210)) return 1;		//cyan
	 else if ((c1 >= 210) && (c1 < 270) && (c2 >= 210) && (c2 < 270)) return 1;		//blue
	 else if ((c1 >= 270) && (c1 < 300) && (c2 >= 270) && (c2 < 300)) return 1;		//purple
	 else if ((c1 >= 300) && (c1 <= 360) && (c2 >= 300) && (c2 <= 360)) return 1;	//pink
	 else return 0;

	 return 0;
 }

 int combine_label(int *object_array, int size, int label, int hsv, double ic, double jc, int pos){
	 //purpose: to populate the robot_a, robot_b, obstacles' array with cruicial info

	 //local variables
	 int i, n;
											
	 if (pos){									//--1intended to populate robot_a, robot_b arrays 
		 n = 4 * (pos - 1) + 1;					//pos being the indicator for location to where the data needs to be stored
		 object_array[n] = hsv;					//						  so as to ensure that the 1st colour stored represents the head
		 object_array[n + 1] = label;			//												   2nd colour stored represents the tail
		 object_array[n + 2] = (int)ic;
		 object_array[n + 3] = (int)jc;
	 }
	 else{
		for (i = 0; i < size; i++){				//--2intended to populate obstacles' array
				 n = 4 * i + 1;					
				 if (object_array[n] == 0){		//order is not of concern
					 object_array[n] = hsv;
					 object_array[n + 1] = label;
					 object_array[n + 2] = (int)ic;
					 object_array[n + 3] = (int)jc;
					 return 0;
				 }
			 }
	 }
	 

	 return 0;
 }

 int filter_objects(int &nlabels, image &label, int *x_size1, int *x_size2, int *y_size1, int *y_size2, image &b, image &rgb0, int *c1, int *c2, int *c3, int *c4, int *robot_a, int *robot_b, int *obstacles, int *obstacles_size, int &size_r, int &size_o){
	 //purpose: to filter/identify the objects on screen

	 //local variables
	 int w, h, k;
	 int rc, gc, bc, hc, sc, vc;
	 double ic, jc;
	 int pos_obs = 0;
	 
	 for (k = 1; k < nlabels; k++){
		//--1 start by acquiring info about sizes of objects
		size_label(label, k, x_size1, x_size2, y_size1, y_size2);				 
		w = x_size2[k] - x_size1[k];		h = y_size2[k] - y_size1[k]; 
		
		//--2 acquire centroid i,j data from label
		centroid(b, label, k, ic, jc);

		//--3 using centroid i,j get rgb at given location, convert rgb to hsv
		rgb_location(rgb0, ic, jc, rc, gc, bc);
		rgb_2_hsv(rc, gc, bc, hc, sc, vc);

		//--4 with necessary info collected, start identifying objects
		if ((w / h) < (3 / 2) || (w / h) > (2 / 3)){					//ensures type of objects disgarded are of certain proportions, aim to catch all circular objects
			if (w > 30 && h > 30){										//ensures type of objects disgarded are of a certain size, disgard wheels
				//identify robot_a (our robot)
				if (same_hue(c1[4], hc)) {	combine_label(robot_a, size_r, k, hc, ic, jc,1);	}
				if (same_hue(c2[4], hc)) {	combine_label(robot_a, size_r, k, hc, ic, jc,2);	}
				
				//identify robot_b (enemy robot)
				if (same_hue(c3[4], hc)) {	combine_label(robot_b, size_r, k, hc, ic, jc,1);	}
				if (same_hue(c4[4], hc)) {	combine_label(robot_b, size_r, k, hc, ic, jc,2);	}
				
				//identify all obstacles
				if (vc <= 25) {
					 pos_obs++;
					 combine_label(obstacles, size_o, k, vc, ic, jc,0);
					 if (w>=h){update_obstacle_size(obstacles, obstacles_size, size_o, w, pos_obs);}
					 else if (w<h){ update_obstacle_size(obstacles, obstacles_size, size_o, h, pos_obs); }
					 
				}

			}
		}
	}


	 return 0;
}

 int filter_objects_update(int &nlabels, image &label, int *x_size1, int *x_size2, int *y_size1, int *y_size2, image &b, image &rgb0, int *c1, int *c2, int *c3, int *c4, int *robot_a, int *robot_b, int *obstacles, int *obstacles_size, int &size_r, int &size_o){
	 //purpose: for re-filtering, to be called only once inside loop

	 //re-initialise obstacles array
	 initialize_obstacle_arrays(obstacles, obstacles_size, size_o);

	 //local varaibles
	 int w, h, k;
	 int rc, gc, bc, hc, sc, vc;
	 double ic, jc;
	 int pos_obs;

	 for (k = 1; k < nlabels; k++){
		 //--1 start by acquiring info about sizes of objects
		 size_label(label, k, x_size1, x_size2, y_size1, y_size2);
		 w = x_size2[k] - x_size1[k];		h = y_size2[k] - y_size1[k]; 

		 //--2 acquire centroid i,j data from label
		 centroid(b, label, k, ic, jc);

		 //--3 using centroid i,j get rgb at given location, convert rgb to hsv
		 rgb_location(rgb0, ic, jc, rc, gc, bc);
		 rgb_2_hsv(rc, gc, bc, hc, sc, vc);

		 //--4 with necessary info collected, start identifying objects
		 if ((w / h) < (3 / 2) || (w / h) > (2 / 3)){						//ensures type of objects disgarded are of certain proportions, aim to catch all circular objects
			 if (w > 30 && h > 30){											//ensures type of objects disgarded are of a certain size, disgard wheels
				 //update robot_a (our robot)
				 if (same_hue(c1[4], hc)) {		update_label(robot_a, size_r, k, hc, ic, jc);		}
				 if (same_hue(c2[4], hc)) {		update_label(robot_a, size_r, k, hc, ic, jc);		}
				 
				 //update robot_b (enemy robot)
				 if (same_hue(c3[4], hc)) {		update_label(robot_b, size_r, k, hc, ic, jc);		}
				 if (same_hue(c4[4], hc)) {		update_label(robot_b, size_r, k, hc, ic, jc);		}
				 
				 //update all obstacles
				 if (vc <= 25) {
					 pos_obs++;
					 combine_label(obstacles, size_o, k, vc, ic, jc,0);
					 if (w >= h){ update_obstacle_size(obstacles, obstacles_size, size_o, w, pos_obs); }
					 else if (w<h){ update_obstacle_size(obstacles, obstacles_size, size_o, h,pos_obs); }
				 }

			 }
		 }
	 }


	 return 0;
 }

 int filter_objects_check(int &x_robot, int &y_robot, int &d_SF, int &nlabels_actual, int &x_opp, int &y_opp, int *x_obs_array, int *y_obs_array, int &nlabels, image &label, int *x_size1, int *x_size2, int *y_size1, int *y_size2, image &b, image &rgb0, int *c1, int *c2, int *c3, int *c4, int *robot_a, int *robot_b, int *obstacles, int *obstacles_size, int &size_r, int &size_o){
	 //purpose: to attempt to re-identify all objects, in the scenario where objects come back to screen
	 //aka recovery

	 //local variables
	 int i;

	 //if object gets too close to boundaries
	 if (x_robot <= (d_SF - 0) || x_robot >= (640 - d_SF))		filter_objects_update(nlabels, label, x_size1, x_size2, y_size1, y_size2, b, rgb0, c1, c2, c3, c4, robot_a, robot_b, obstacles, obstacles_size, size_r, size_o);
	 if (y_robot <= (d_SF - 0) || y_robot >= (480 - d_SF))	filter_objects_update(nlabels, label, x_size1, x_size2, y_size1, y_size2, b, rgb0, c1, c2, c3, c4, robot_a, robot_b, obstacles, obstacles_size, size_r, size_o);
	 if (nlabels <= nlabels_actual) filter_objects_update(nlabels, label, x_size1, x_size2, y_size1, y_size2, b, rgb0, c1, c2, c3, c4, robot_a, robot_b, obstacles, obstacles_size, size_r, size_o);

	 //check if any x,y were combined
	 //with robot_b/enemy robot
	 if (x_robot == x_opp && y_robot == y_opp) filter_objects_update(nlabels, label, x_size1, x_size2, y_size1, y_size2, b, rgb0, c1, c2, c3, c4, robot_a, robot_b, obstacles, obstacles_size, size_r, size_o);
	 //with obstacles
	 for (i = 1; i <= size_o; i++){
		 if (x_robot == x_obs_array[i] && y_robot == y_obs_array[i]) filter_objects_update(nlabels, label, x_size1, x_size2, y_size1, y_size2, b, rgb0, c1, c2, c3, c4, robot_a, robot_b, obstacles, obstacles_size, size_r, size_o);
	 }


	 return 0;
 }

 int update_label(int *object_array, int size, int label, int hsv, double ic, double jc){
	 //purpose: to update object info, via hsv identification

	 //local variables
	 int i, n;

	 for (i = 0; i < size; i++){
		 n = 4 * i + 1;
		 if (object_array[n] == hsv){
			 object_array[n + 1] = label;
			 object_array[n + 2] = (int)ic;
			 object_array[n + 3] = (int)jc;
			 return 0;
		 }
	 }

	 return 0;
 }

 int check_label(int *object_array, int size, int label){
	 //purpose: act as helper function for something akin to eliminate_object (reset wheel label to background label -> 0) wasn not implemented
		
	 //local variables
	 int i, n;

	 for (i = 0; i < size; i++){
		 n = 4 * i + 1;
		 if (object_array[n + 1] == label) return 1;
	 }

	 return 0;
 }


 //Tracking Objects

 int search_object(i2byte &nlabel, image &label, int is, int js)
	 //purpose: to search for an object, from previous location provided (in spiral pattern), updated label provided
	 // search for a labeled object in an outward spiral pattern
	 // and inital search location (is,js)
 {
	 i2byte *pl;
	 double r, rmax, dr, s, smax, ds, theta;
	 int i, j;

	 // pointer to a label image
	 pl = (i2byte *)label.pdata;

	 // check if an object exists at the current location
	 nlabel = *(pl + js*label.width + is);
	 if (nlabel != 0) return 0;

	 rmax = 60.0; // maximum radius of search (pixels)
	 dr = 3.0; // radius divisions (pixels)
	 ds = 3.0; // arc-length divisions (pixels)

	 // search for a labeled object in an outward concentic ring pattern
	 for (r = 1.0; r <= rmax; r += dr) {
		 smax = 2 * 3.1416*r; // maximum arc length
		 for (s = 0; s <= smax; s += ds) {
			 theta = s / r; // s = r*theta
			 i = (int)(is + r*cos(theta));
			 j = (int)(js + r*sin(theta));

			 // limit (i,j) from going off the image
			 if (i < 0) i = 0;
			 if (i > label.width - 1) i = label.width - 1;
			 if (j < 0) j = 0;
			 if (j > label.height - 1) j = label.height - 1;

			 //			*( b.pdata + j*b.width + i ) = 128; // check pattern

			 // check if there is an object at location (i,j)
			 nlabel = *(pl + j*label.width + i);
			 if (nlabel != 0) return 0;
		 }
	 }

	 return 0; // no errors
 }

 int track_object(int *r_array, int size, image &label, image &b, image &rgb0){
	 //purpose: to track an object (robot_a/our robot, robot_b/enemy robot, obstacles) using the seach alg/fn provided by Prof. Gordon
	
	 //local variables
	 int i, n, c_hsv;
	 i2byte c_label;
	 double c_ic, c_jc;

	 for (i = 0; i < (size); i++){
		 //--1 retrieve current stored info, for some object
		 n = 4 * i + 1;
		 c_hsv = r_array[n];
		 c_label = (i2byte)r_array[n + 1];
		 c_ic = (double)r_array[n + 2];
		 c_jc = (double)r_array[n + 3];

		 //--2 call search function using previos i,j			--> label to be updated (if changed)
		 search_object(c_label, label, (int)c_ic, (int)c_jc);
		 r_array[n + 1] = (int)c_label;

		 //--3 call centroid function using updated label		--> provide new i,j centroid locations for object (if changed)
		 centroid(b, label, (int)c_label, c_ic, c_jc);
		 r_array[n + 2] = (int)c_ic;
		 r_array[n + 3] = (int)c_jc;

		 //--3.2 (for debugging purposes)
		 //draw_point_rgb(rgb0, (int)c_ic, (int)c_jc, 57, 255, 20);
	 }

	 return 0;
 }


 // Control Functions
 int align_full_throttle(double phi_robot, double phi_goal, int &pw_l, int &pw_r)
 {
	 double e_orientation;
	 double ua;
	 e_orientation = phi_goal - phi_robot;

	 if (e_orientation <= 0)
	 {//Rotate Clockwise 
		 ua =  1500 - 500 ;

		 pw_l = ua;
		 pw_r = ua;

	 }

	 else if (e_orientation > 0)
	 {
		 //Rotate Anti-Clockwise 
		 ua = 1500 + 500 ;

		 pw_l = ua;
		 pw_r = ua;

	 }

	 return 0;
 }

 int align_closed_loop(double phi_robot,double phi_goal,double dt,int &pw_l, int &pw_r,double &ep,double &ei)
 {
	 // PID Control Function for controlling angular velocity of robot about z-axis
	 double e_orientation;
	 e_orientation = phi_goal - phi_robot;

	 double e, ed, u, ua;
	 double kp, ki, kd;

	 //Target Lock Gains 
	 kp = 0.00283; // proportional gain
	 ki = 0.00083; // integral gain
	 kd = 0.0; // derivative gain


	 e = e_orientation;

	 ed = (e - ep) / dt;
	 ep = e;

	 ei += e*dt;

	 u = (kp*e + ki*ei + kd*ed);

	 // saturate control input
	 if (u > 1.0)  u = 1.0;
	 if (u < -1.0) u = -1.0;
 
	 //-1 for clockwise and 1 for anticlockwise
	 ua = 1500 + 500 * u;

	 pw_l = ua;
	 pw_r = ua;
	 
	 return 0;
 }

 /// Align robot towards goal if error is more than tolerance- Less Stable for nearby objects 
 int align_fast(double phi_robot, double phi_goal, int &pw_l, int &pw_r, double tolerance)
 {
	 double e_orientation;
	 double ua;
	 e_orientation = phi_goal - phi_robot;

	 if (e_orientation >= tolerance || e_orientation <= -tolerance)
	 {
		 align_full_throttle(phi_robot, phi_goal, pw_l, pw_r);
	 }
	 return 0;
 }


 /// locks robot towards goal if error is more than tolerance _ Stable for Nearby objects
 int align_lock(double phi_robot, double phi_goal, double dt, int &pw_l, int &pw_r, double &ep, double &ei, double tolerance)
 {
	 double e_orientation;
	 double ua;
	 e_orientation = phi_goal - phi_robot;

	 if (e_orientation >= tolerance || e_orientation <= -tolerance)
	 {
		 align_closed_loop(phi_robot, phi_goal, dt, pw_l, pw_r, ep, ei);
	 }
	 return 0;
 }

 int move_forward(int &pw_l, int &pw_r,double &ep,double &ei)
 {
	 ep = 0; // Clear prev errors for PID Control as differential drive robot has mobility of 2 and 3 states
	 ei = 0; // i.e it can only rotate or move at time , x,y,phi all 3 of them can't be controlled together
	 pw_l = 1000;
	 pw_r = 2000;
	 return 0;
 }

 int move_backward(int &pw_l, int &pw_r, double &ep, double &ei)
 {
	 ep = 0; // Clear prev errors for PID Control as differential drive robot has mobility of 2 and 3 states
	 ei = 0; // i.e it can only rotate or move at time , x,y,phi all 3 of them can't be controlled together
	 pw_l = 2000;
	 pw_r = 1000;
	 return 0;
 }

 int stop_robot(int &pw_l, int &pw_r)
 {
	 pw_l = 1500;
	 pw_r = 1500;
	 return 0;
 }
