////////////////////////////////////////////////////////////////////////////////////         
//Final Project			-A Ball moving on 3D track avoiding obstacles to reach finish line.
//
// This program allows the user to move a car on a plane and view the rankings.
// It is built on 3-D surface.
//
// Interaction:
 /*Press w to accelerate / continuously press on w to increase acceleration
 Press a to move left.
 Press d to move right.*/
// Press the x, X, y, Y, z, Z keys to rotate the viewpoint.
//
//////////////////////////////////////////////////////////////////////////////////// 

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include "getBMP.h"
#define PI 3.14159265


using namespace std;
/******************************** Begin globals ***********************************/
// Initial control points.
static float start[6][6][3] =
{ { { -5.0, 0.0, 25.0 },{ -3.0, 0.0, 25.0 },{ -0.25, 0.0, 25.0 },{ 0.25, 0.0, 25.0 },{ 3.0, 0.0, 25.0 },{ 5.0, 0.0, 25.0 } },
{ { -5.0, 0.0, 22.0 },{ -3.0, 0.0, 22.0 },{ -0.25, 0.0, 22.0 },{ 0.25, 0.0, 22.0 },{ 3.0, 0.0, 22.0 },{ 5.0, 0.0, 22.0 } },
{ { -5.0, 0.0, 19.0 },{ -3.0, 0.0, 19.0 },{ -0.25, 0.0, 19.0 },{ 0.25, 0.0, 19.0 },{ 3.0, 0.0, 19.0 },{ 5.0, 0.0, 19.0 } },
{ { -5.0, 0.0, 16.0 },{ -3.0, 0.0, 16.0 },{ -0.25, 0.0, 16.0 },{ 0.25, 0.0, 16.0 },{ 3.0, 0.0, 16.0 } ,{ 5.0, 0.0, 16.0 } },
{ { -5.0, 0.0, 13.0 },{ -3.0, 0.0, 13.0 },{ -0.25, 0.0, 13.0 },{ 0.25, 0.0, 13.0 },{ 3.0, 0.0, 13.0 },{ 5.0, 0.0, 13.0 } },
{ { -5.0, 0.0, 10.0 },{ -3.0, 0.0, 10.0 },{ -0.25, 0.0, 10.0 },{ 0.25, 0.0, 10.0 },{ 3.0, 0.0, 10.0 },{ 5.0, 0.0, 10.0 } },

};

static float controlPoints[6][6][3] =
{
	{ { -5.0, 0.0, 10.0 },{ -3.0, 0.0, 10.0 },{ -0.25, 0.0, 10.0 },{ 0.25, 0.0, 10.0 },{ 3.0, 0.0, 10.0 },{ 5.0, 0.0, 10.0 } },
	{ { -5.0, 0.0, 7.0 },{ -3.0, 0.0, 7.0 },{ -0.25, 0.0, 7.0 },{ 0.25, 0.0, 7.0 },{ 3.0, 0.0, 7.0 },{ 5.0, 0.0, 7.0 } },
	{ { -5.0, 0.0, 4.0 },{ -3.0, 0.0, 4.0 },{ -0.25, 0.0, 4.0 },{ 0.25, 0.0, 4.0 },{ 3.0, 0.0, 4.0 },{ 5.0, 0.0, 4.0 } },
	{ { -5.0, 0.0, 1.0 },{ -3.0, 0.0, 1.0 },{ -0.25, 0.0, 1.0 },{ 0.25, 0.0, 1.0 },{ 3.0, 0.0, 1.0 },{ 5.0, 0.0, 1.0 } },
	{ { -5.0, 0.0, -2.0 },{ -3.0, 0.0, -2.0 },{ -0.25, 0.0, -2.0 },{ 0.25, 0.0, -2.0 },{ 3.0, 0.0, -2.0 },{ 5.0, 0.0, -2.0 } },
	{ { -5.0, 0.0, -5.0 },{ -3.0, 0.0, -5.0 },{ -0.25, 0.0, -5.0 },{ 0.25, 0.0, -5.0 },{ 3.0, 0.0, -5.0 },{ 5.0, 0.0, -5.0 } },
};

static float controlPoints_1[6][6][3] =
{
	{ { -5.0, 0.0, -5.0 },{ -3.0, 0.0, -5.0 },{ -0.25, 0.0, -5.0 },{ 0.25, 0.0, -5.0 },{ 3.0, 0.0, -5.0 },{ 5.0, 0.0, -5.0 } },
	{ { -5.0, 0.0, -8.0 },{ -3.0, 0.0, -8.0 },{ -0.25, 0.0, -8.0 },{ 0.25, 0.0, -8.0 },{ 3.0, 0.0, -8.0 },{ 5.0, 0.0, -8.0 } },
	{ { -5.0, 0.0, -11.0 },{ -3.0, 0.0, -11.0 },{ -0.25, 0.0, -11.0 },{ 0.25, 0.0, -11.0 },{ 3.0, 0.0, -11.0 },{ 5.0, 0.0, -11.0 } },
	{ { -5.0, 0.0, -14.0 },{ -3.0, 0.0, -14.0 },{ -0.25, 0.0, -14.0 },{ 0.25, 0.0, -14.0 },{ 3.0, 0.0, -14.0 },{ 5.0, 0.0, -14.0 } },
	{ { -5.0, 0.0, -17.0 },{ -3.0, 0.0, -17.0 },{ -0.25, 0.0, -17.0 },{ 0.25, 0.0, -17.0 },{ 3.0, 0.0, -17.0 },{ 5.0, 0.0, -17.0 } },
	{ { -5.0, 0.0, -20.0 },{ -3.0, 0.0, -20.0 },{ -0.25, 0.0, -20.0 },{ 0.25, 0.0, -20.0 },{ 3.0, 0.0, -20.0 },{ 5.0, 0.0, -20.0 } },
};

static float controlPoints_2[6][6][3] =
{
	{ { -5.0, 0.0, -20.0 },{ -3.0, 0.0, -20.0 },{ -0.25, 0.0, -20.0 },{ 0.25, 0.0, -20.0 },{ 3.0, 0.0, -20.0 },{ 5.0, 0.0, -20.0 } },
	{ { -5.0, 0.0, -23.0 },{ -3.0, 0.0, -23.0 },{ -0.25, 0.0, -23.0 },{ 0.25, 0.0, -23.0 },{ 3.0, 0.0, -23.0 },{ 5.0, 0.0, -23.0 } },
	{ { -5.0, 0.0, -26.0 },{ -3.0, 0.0, -26.0 },{ -0.25, 0.0, -26.0 },{ 0.25, 0.0, -26.0 },{ 3.0, 0.0, -26.0 },{ 5.0, 0.0, -26.0 } },
	{ { -5.0, 0.0, -29.0 },{ -3.0, 0.0, -29.0 },{ -0.25, 0.0, -29.0 },{ 0.25, 0.0, -29.0 },{ 3.0, 0.0, -29.0 },{ 5.0, 0.0, -29.0 } },
	{ { -5.0, 0.0, -32.0 },{ -3.0, 0.0, -32.0 },{ -0.25, 0.0, -32.0 },{ 0.25, 0.0, -32.0 },{ 3.0, 0.0, -32.0 },{ 5.0, 0.0, -32.0 } },
	{ { -5.0, 0.0, -35.0 },{ -3.0, 0.0, -35.0 },{ -0.25, 0.0, -35.0 },{ 0.25, 0.0, -35.0 },{ 3.0, 0.0, -35.0 },{ 5.0, 0.0, -35.0 } },
};
static float controlPoints_3[6][6][3] =
{
	{ { -5.0, 0.0, -35.0 },{ -3.0, 0.0, -35.0 },{ -0.25, 0.0, -35.0 },{ 0.25, 0.0, -35.0 },{ 3.0, 0.0, -35.0 },{ 5.0, 0.0, -35.0 } },
	{ { -5.0, -0.4, -38.0 },{ -3.0, -0.4, -38.0 },{ -0.25, -0.4, -38.0 },{ 0.25, -0.4, -38.0 },{ 3.0, -0.4, -38.0 },{ 5.0, -0.4, -38.0 } },
	{ { -5.0, -0.8, -41.0 },{ -3.0, -0.8, -41.0 },{ -0.25, -0.8, -41.0 },{ 0.25, -0.8, -41.0 },{ 3.0, -0.8, -41.0 },{ 5.0, -0.8, -41.0 } },
	{ { -5.0, -1.2, -44.0 },{ -3.0, -1.2, -44.0 },{ -0.25, -1.2, -44.0 },{ 0.25, -1.2, -44.0 },{ 3.0, -1.2, -44.0 },{ 5.0, -1.2, -44.0 } },
	{ { -5.0, -1.6, -47.0 },{ -3.0, -1.6, -47.0 },{ -0.25, -1.6, -47.0 },{ 0.25, -1.6, -47.0 },{ 3.0, -1.6, -47.0 },{ 5.0, -1.6, -47.0 } },
	{ { -5.0, -2.0, -50.0 },{ -3.0, -2.0, -50.0 },{ -0.25, -2.0, -50.0 },{ 0.25, -2.0, -50.0 },{ 3.0, -2.0, -50.0 },{ 5.0, -2.0, -50.0 } },
};
static float controlPoints_4[6][6][3] =
{
	{ { -5.0, -2.0, -50.0 },{ -3.0, -2.0, -50.0 },{ -0.25, -2.0, -50.0 },{ 0.25, -2.0, -50.0 },{ 3.0, -2.0, -50.0 },{ 5.0, -2.0, -50.0 } },
	{ { -5.0, -1.6, -53.0 },{ -3.0, -1.6, -53.0 },{ -0.25, -1.6, -53.0 },{ 0.25, -1.6, -53.0 },{ 3.0, -1.6, -53.0 },{ 5.0, -1.6, -53.0 } },
	{ { -5.0, -1.2, -56.0 },{ -3.0, -1.2, -56.0 },{ -0.25, -1.2, -56.0 },{ 0.25, -1.2, -56.0 },{ 3.0, -1.2, -56.0 },{ 5.0, -1.2, -56.0 } },
	{ { -5.0, -0.8, -59.0 },{ -3.0, -0.8, -59.0 },{ -0.25, -0.8, -59.0 },{ 0.25, -0.8, -59.0 },{ 3.0, -0.8, -59.0 },{ 5.0, -0.8, -59.0 } },
	{ { -5.0, -0.4, -62.0 },{ -3.0, -0.4, -62.0 },{ -0.25, -0.4, -62.0 },{ 0.25, -0.4, -62.0 },{ 3.0, -0.4, -62.0 },{ 5.0, -0.4, -62.0 } },
	{ { -5.0, 0.0, -65.0 },{ -3.0, 0.0, -65.0 },{ -0.25, 0.0, -65.0 },{ 0.25, 0.0, -65.0 },{ 3.0, 0.0, -65.0 },{ 5.0, 0.0, -65.0 } },
};
static float controlPoints_5[6][6][3] =
{
	{ { -5.0, 0.0, -65.0 },{ -3.0, 0.0, -65.0 },{ -0.25, 0.0, -65.0 },{ 0.25, 0.0, -65.0 },{ 3.0, 0.0, -65.0 },{ 5.0, 0.0, -65.0 } },
	{ { -5.0, 0.0, -68.0 },{ -3.0, 0.0, -68.0 },{ -0.25, 0.0, -68.0 },{ 0.25, 0.0, -68.0 },{ 3.0, 0.0, -68.0 },{ 5.0, 0.0, -68.0 } },
	{ { -5.0, 0.0, -71.0 },{ -3.0, 0.0, -71.0 },{ -0.25, 0.0, -71.0 },{ 0.25, 0.0, -71.0 },{ 3.0, 0.0, -71.0 },{ 5.0, 0.0, -71.0 } },
	{ { -5.0, 0.0, -74.0 },{ -3.0, 0.0, -74.0 },{ -0.25, 0.0, -74.0 },{ 0.25, 0.0, -74.0 },{ 3.0, 0.0, -74.0 },{ 5.0, 0.0, -74.0 } },
	{ { -5.0, 0.0, -77.0 },{ -3.0, 0.0, -77.0 },{ -0.25, 0.0, -77.0 },{ 0.25, 0.0, -77.0 },{ 3.0, 0.0, -77.0 },{ 5.0, 0.0, -77.0 } },
	{ { -5.0, 0.0, -80.0 },{ -3.0, 0.0, -80.0 },{ -0.25, 0.0, -80.0 },{ 0.25, 0.0, -80.0 },{ 3.0, 0.0, -80.0 },{ 5.0, 0.0, -80.0 } },
};

static float controlPoints_6[6][6][3] =
{
	{ { -5.0, 0.0, -80.0 },{ -3.0, 0.0, -80.0 },{ -0.25, 0.0, -80.0 },{ 0.25, 0.0, -80.0 },{ 3.0, 0.0, -80.0 },{ 5.0, 0.0, -80.0 } },
	{ { -5.0, 0.0, -83.0 },{ -3.0, 0.0, -83.0 },{ -0.25, 0.0, -83.0 },{ 0.25, 0.0, -83.0 },{ 3.0, 0.0, -83.0 },{ 5.0, 0.0, -83.0 } },
	{ { -5.0, 0.0, -86.0 },{ -3.0, 0.0, -86.0 },{ -0.25, 0.0, -86.0 },{ 0.25, 0.0, -86.0 },{ 3.0, 0.0, -86.0 },{ 5.0, 0.0, -86.0 } },
	{ { -5.0, 0.0, -89.0 },{ -3.0, 0.0, -89.0 },{ -0.25, 0.0, -89.0 },{ 0.25, 0.0, -89.0 },{ 3.0, 0.0, -89.0 },{ 5.0, 0.0, -89.0 } },
	{ { -5.0, 0.0, -92.0 },{ -3.0, 0.0, -92.0 },{ -0.25, 0.0, -92.0 },{ 0.25, 0.0, -92.0 },{ 3.0, 0.0, -92.0 },{ 5.0, 0.0, -92.0 } },
	{ { -5.0, 0.0, -95.0 },{ -3.0, 0.0, -95.0 },{ -0.25, 0.0, -95.0 },{ 0.25, 0.0, -95.0 },{ 3.0, 0.0, -95.0 },{ 5.0, 0.0, -95.0 } },
};
static float controlPoints_7[6][6][3] =
{
	{ { -5.0, 0.0, -95.0 },{ -3.0, 0.0, -95.0 },{ -0.25, 0.0, -95.0 },{ 0.25, 0.0, -95.0 },{ 3.0, 0.0, -95.0 },{ 5.0, 0.0, -95.0 } },
	{ { -5.0, 0.0, -98.0 },{ -3.0, 0.0, -98.0 },{ -0.25, 0.0, -98.0 },{ 0.25, 0.0, -98.0 },{ 3.0, 0.0, -98.0 },{ 5.0, 0.0, -98.0 } },
	{ { -5.0, 0.0, -101.0 },{ -3.0, 0.0, -101.0 },{ -0.25, 0.0, -101.0 },{ 0.25, 0.0, -101.0 },{ 3.0, 0.0, -101.0 },{ 5.0, 0.0, -101.0 } },
	{ { -5.0, 0.0, -104.0 },{ -3.0, 0.0, -104.0 },{ -0.25, 0.0, -104.0 },{ 0.25, 0.0, -104.0 },{ 3.0, 0.0, -104.0 },{ 5.0, 0.0, -104.0 } },
	{ { -5.0, 0.0, -107.0 },{ -3.0, 0.0, -107.0 },{ -0.25, 0.0, -107.0 },{ 0.25, 0.0, -107.0 },{ 3.0, 0.0, -107.0 },{ 5.0, 0.0, -107.0 } },
	{ { -5.0, 0.0, -110.0 },{ -3.0, 0.0, -110.0 },{ -0.25, 0.0, -110.0 },{ 0.25, 0.0, -110.0 },{ 3.0, 0.0, -110.0 },{ 5.0, 0.0, -110.0 } },
};
static float controlPoints_8[6][6][3] =
{
	{ { -5.0, 0.0, -110.0 },{ -3.0, 0.0, -110.0 },{ -0.25, 0.0, -110.0 },{ 0.25, 0.0, -110.0 },{ 3.0, 0.0, -110.0 },{ 5.0, 0.0, -110.0 } },
	{ { -5.0, 0.0, -113.0 },{ -3.0, 0.0, -113.0 },{ -0.25, 0.0, -113.0 },{ 0.25, 0.0, -113.0 },{ 3.0, 0.0, -113.0 },{ 5.0, 0.0, -113.0 } },
	{ { -5.0, 0.0, -116.0 },{ -3.0, 0.0, -116.0 },{ -0.25, 0.0, -116.0 },{ 0.25, 0.0, -116.0 },{ 3.0, 0.0, -116.0 },{ 5.0, 0.0, -116.0 } },
	{ { -5.0, 0.0, -119.0 },{ -3.0, 0.0, -119.0 },{ -0.25, 0.0, -119.0 },{ 0.25, 0.0, -119.0 },{ 3.0, 0.0, -119.0 },{ 5.0, 0.0, -119.0 } },
	{ { -5.0, 0.0, -122.0 },{ -3.0, 0.0, -122.0 },{ -0.25, 0.0, -122.0 },{ 0.25, 0.0, -122.0 },{ 3.0, 0.0, -122.0 },{ 5.0, 0.0, -122.0 } },
	{ { -5.0, 0.0, -125.0 },{ -3.0, 0.0, -125.0 },{ -0.25, 0.0, -125.0 },{ 0.25, 0.0, -125.0 },{ 3.0, 0.0, -125.0 },{ 5.0, 0.0, -125.0 } },
};
static float controlPoints_9[6][6][3] =
{
	{ { -5.0, 0.0, -125.0 },{ -3.0, 0.0, -125.0 },{ -0.25, 0.0, -125.0 },{ 0.25, 0.0, -125.0 },{ 3.0, 0.0, -125.0 },{ 5.0, 0.0, -125.0 } },
	{ { -5.0, 0.0, -128.0 },{ -3.0, 0.0, -128.0 },{ -0.25, 0.0, -128.0 },{ 0.25, 0.0, -128.0 },{ 3.0, 0.0, -128.0 },{ 5.0, 0.0, -128.0 } },
	{ { -5.0, 0.0, -131.0 },{ -3.0, 0.0, -131.0 },{ -0.25, 0.0, -131.0 },{ 0.25, 0.0, -131.0 },{ 3.0, 0.0, -131.0 },{ 5.0, 0.0, -131.0 } },
	{ { -5.0, 0.0, -134.0 },{ -3.0, 0.0, -134.0 },{ -0.25, 0.0, -134.0 },{ 0.25, 0.0, -134.0 },{ 3.0, 0.0, -134.0 },{ 5.0, 0.0, -134.0 } },
	{ { -5.0, 0.0, -137.0 },{ -3.0, 0.0, -137.0 },{ -0.25, 0.0, -137.0 },{ 0.25, 0.0, -137.0 },{ 3.0, 0.0, -137.0 },{ 5.0, 0.0, -137.0 } },
	{ { -5.0, 0.0, -140.0 },{ -3.0, 0.0, -140.0 },{ -0.25, 0.0, -140.0 },{ 0.25, 0.0, -140.0 },{ 3.0, 0.0, -140.0 },{ 5.0, 0.0, -140.0 } },
};

static float controlpoints_10[6][6][3] =
{
	{ { -5.0, 0.0, -140.0 },{ -3.0, 0.0, -140.0 },{ -0.25, 0.0, -140.0 },{ 0.25, 0.0, -140.0 },{ 3.0, 0.0, -140.0 },{ 5.0, 0.0, -140.0 } },
	{ { -5.0, 0.0, -143.0 },{ -3.0, 0.0, -143.0 },{ -0.25, 0.0, -143.0 },{ 0.25, 0.0, -143.0 },{ 3.0, 0.0, -143.0 },{ 5.0, 0.0, -143.0 } },
	{ { -5.0, 0.0, -146.0 },{ -3.0, 0.0, -146.0 },{ -0.25, 0.0, -146.0 },{ 0.25, 0.0, -146.0 },{ 3.0, 0.0, -146.0 },{ 5.0, 0.0, -146.0 } },
	{ { -5.0, 0.0, -149.0 },{ -3.0, 0.0, -149.0 },{ -0.25, 0.0, -149.0 },{ 0.25, 0.0, -149.0 },{ 3.0, 0.0, -149.0 },{ 5.0, 0.0, -149.0 } },
	{ { -5.0, 0.0, -152.0 },{ -3.0, 0.0, -152.0 },{ -0.25, 0.0, -152.0 },{ 0.25, 0.0, -152.0 },{ 3.0, 0.0, -152.0 },{ 5.0, 0.0, -152.0 } },
	{ { -5.0, 0.0, -155.0 },{ -3.0, 0.0, -155.0 },{ -0.25, 0.0, -155.0 },{ 0.25, 0.0, -155.0 },{ 3.0, 0.0, -155.0 },{ 5.0, 0.0, -155.0 } },
};
static float controlpoints_11[6][6][3] =
{
	{ { -5.0, 0.0, -155.0 },{ -3.0, 0.0, -155.0 },{ -0.25, 0.0, -155.0 },{ 0.25, 0.0, -155.0 },{ 3.0, 0.0, -155.0 },{ 5.0, 0.0, -155.0 } },
	{ { -5.0, 0.0, -158.0 },{ -3.0, 0.0, -158.0 },{ -0.25, 0.0, -158.0 },{ 0.25, 0.0, -158.0 },{ 3.0, 0.0, -158.0 },{ 5.0, 0.0, -158.0 } },
	{ { -5.0, 0.0, -161.0 },{ -3.0, 0.0, -161.0 },{ -0.25, 0.0, -161.0 },{ 0.25, 0.0, -161.0 },{ 3.0, 0.0, -161.0 },{ 5.0, 0.0, -161.0 } },
	{ { -5.0, 0.0, -164.0 },{ -3.0, 0.0, -164.0 },{ -0.25, 0.0, -164.0 },{ 0.25, 0.0, -164.0 },{ 3.0, 0.0, -164.0 },{ 5.0, 0.0, -164.0 } },
	{ { -5.0, 0.0, -167.0 },{ -3.0, 0.0, -167.0 },{ -0.25, 0.0, -167.0 },{ 0.25, 0.0, -167.0 },{ 3.0, 0.0, -167.0 },{ 5.0, 0.0, -167.0 } },
	{ { -5.0, 0.0, -170.0 },{ -3.0, 0.0, -170.0 },{ -0.25, 0.0, -170.0 },{ 0.25, 0.0, -170.0 },{ 3.0, 0.0, -170.0 },{ 5.0, 0.0, -170.0 } },
};
static float controlpoints_12[6][6][3] =
{
	{ { -5.0, 0.0, -170.0 },{ -3.0, 0.0, -170.0 },{ -0.25, 0.0, -170.0 },{ 0.25, 0.0, -170.0 },{ 3.0, 0.0, -170.0 },{ 5.0, 0.0, -170.0 } },
	{ { -5.0, 0.4, -173.0 },{ -3.0, 0.4, -173.0 },{ -0.25, 0.4, -173.0 },{ 0.25, 0.4, -173.0 },{ 3.0, 0.4, -173.0 },{ 5.0, 0.4, -173.0 } },
	{ { -5.0, 0.8, -176.0 },{ -3.0, 0.8, -176.0 },{ -0.25, 0.8, -176.0 },{ 0.25, 0.8, -176.0 },{ 3.0, 0.8, -176.0 },{ 5.0, 0.8, -176.0 } },
	{ { -5.0, 1.2, -179.0 },{ -3.0, 1.2, -179.0 },{ -0.25, 1.2, -179.0 },{ 0.25, 1.2, -179.0 },{ 3.0, 1.2, -179.0 },{ 5.0, 1.4, -179.0 } },
	{ { -5.0, 1.6, -182.0 },{ -3.0, 1.6, -182.0 },{ -0.25, 1.6, -182.0 },{ 0.25, 1.6, -182.0 },{ 3.0, 1.6, -182.0 },{ 5.0, 1.6, -182.0 } },
	{ { -5.0, 2.0, -185.0 },{ -3.0, 2.0, -185.0 },{ -0.25, 2.0, -185.0 },{ 0.25, 2.0, -185.0 },{ 3.0, 2.0, -185.0 },{ 5.0, 2.0, -185.0 } },
};
static float controlpoints_13[6][6][3] =
{
	{ { -5.0, 2.0, -185.0 },{ -3.0, 2.0, -185.0 },{ -0.25, 2.0, -185.0 },{ 0.25, 2.0, -185.0 },{ 3.0, 2.0, -185.0 },{ 5.0, 2.0, -185.0 } },
	{ { -5.0, 1.6, -188.0 },{ -3.0, 1.6, -188.0 },{ -0.25, 1.6, -188.0 },{ 0.25, 1.6, -188.0 },{ 3.0, 1.6, -188.0 },{ 5.0, 1.6, -188.0 } },
	{ { -5.0, 1.2, -191.0 },{ -3.0, 1.4, -191.0 },{ -0.25, 1.2, -191.0 },{ 0.25, 1.2, -191.0 },{ 3.0, 1.2, -191.0 },{ 5.0, 1.2, -191.0 } },
	{ { -5.0, 0.8, -194.0 },{ -3.0, 1.2, -194.0 },{ -0.25, 0.8, -194.0 },{ 0.25, 0.8, -194.0 },{ 3.0, 0.8, -194.0 },{ 5.0, 0.8, -194.0 } },
	{ { -5.0, 0.4, -197.0 },{ -3.0, 0.4, -197.0 },{ -0.25, 0.4, -197.0 },{ 0.25, 0.4, -197.0 },{ 3.0, 0.4, -197.0 },{ 5.0, 0.4, -197.0 } },
	{ { -5.0, 0.0, -200.0 },{ -3.0, 0.0, -200.0 },{ -0.25, 0.0, -200.0 },{ 0.25, 0.0, -200.0 },{ 3.0, 0.0, -200.0 },{ 5.0, 0.0, -200.0 } },
};
static float controlpoints_14[6][6][3] =
{
	{ { -5.0, 0.0, -200.0 },{ -3.0, 0.0, -200.0 },{ -0.25, 0.0, -200.0 },{ 0.25, 0.0, -200.0 },{ 3.0, 0.0, -200.0 },{ 5.0, 0.0, -200.0 } },
	{ { -5.0, 0.0, -203.0 },{ -3.0, 0.0, -203.0 },{ -0.25, 0.0, -203.0 },{ 0.25, 0.0, -203.0 },{ 3.0, 0.0, -203.0 },{ 5.0, 0.0, -203.0 } },
	{ { -5.0, 0.0, -206.0 },{ -3.0, 0.0, -206.0 },{ -0.25, 0.0, -206.0 },{ 0.25, 0.0, -206.0 },{ 3.0, 0.0, -206.0 },{ 5.0, 0.0, -206.0 } },
	{ { -5.0, 0.0, -209.0 },{ -3.0, 0.0, -209.0 },{ -0.25, 0.0, -209.0 },{ 0.25, 0.0, -209.0 },{ 3.0, 0.0, -209.0 },{ 5.0, 0.0, -209.0 } },
	{ { -5.0, 0.0, -212.0 },{ -3.0, 0.0, -212.0 },{ -0.25, 0.0, -212.0 },{ 0.25, 0.0, -212.0 },{ 3.0, 0.0, -212.0 },{ 5.0, 0.0, -212.0 } },
	{ { -5.0, 0.0, -215.0 },{ -3.0, 0.0, -215.0 },{ -0.25, 0.0, -215.0 },{ 0.25, 0.0, -215.0 },{ 3.0, 0.0, -215.0 },{ 5.0, 0.0, -215.0 } },
};


static float controlpoints_15[6][6][3] =
{
	{ { -5.0, 0.0, -215.0 },{ -3.0, 0.0, -215.0 },{ -0.25, 0.0, -215.0 },{ 0.25, 0.0, -215.0 },{ 3.0, 0.0, -215.0 },{ 5.0, 0.0, -215.0 } },
	{ { -5.0, 0.0, -218.0 },{ -3.0, 0.0, -218.0 },{ -0.25, 0.0, -218.0 },{ 0.25, 0.0, -218.0 },{ 3.0, 0.0, -218.0 },{ 5.0, 0.0, -218.0 } },
	{ { -5.0, 0.0, -221.0 },{ -3.0, 0.0, -221.0 },{ -0.25, 0.0, -221.0 },{ 0.25, 0.0, -221.0 },{ 3.0, 0.0, -221.0 },{ 5.0, 0.0, -221.0 } },
	{ { -5.0, 0.0, -224.0 },{ -3.0, 0.0, -224.0 },{ -0.25, 0.0, -224.0 },{ 0.25, 0.0, -224.0 },{ 3.0, 0.0, -224.0 },{ 5.0, 0.0, -224.0 } },
	{ { -5.0, 0.0, -227.0 },{ -3.0, 0.0, -227.0 },{ -0.25, 0.0, -227.0 },{ 0.25, 0.0, -227.0 },{ 3.0, 0.0, -227.0 },{ 5.0, 0.0, -227.0 } },
	{ { -5.0, 0.0, -230.0 },{ -3.0, 0.0, -230.0 },{ -0.25, 0.0, -230.0 },{ 0.25, 0.0, -230.0 },{ 3.0, 0.0, -230.0 },{ 5.0, 0.0, -230.0 } },
};

static float controlpoints_16[6][6][3] =
{
	{ { -5.0, 0.0, -230.0 },{ -3.0, 0.0, -230.0 },{ -0.25, 0.0, -230.0 },{ 0.25, 0.0, -230.0 },{ 3.0, 0.0, -230.0 },{ 5.0, 0.0, -230.0 } },
	{ { -5.0, 0.0, -233.0 },{ -3.0, 0.0, -233.0 },{ -0.25, 0.0, -233.0 },{ 0.25, 0.0, -233.0 },{ 3.0, 0.0, -233.0 },{ 5.0, 0.0, -233.0 } },
	{ { -5.0, 0.0, -236.0 },{ -3.0, 0.0, -236.0 },{ -0.25, 0.0, -236.0 },{ 0.25, 0.0, -236.0 },{ 3.0, 0.0, -236.0 },{ 5.0, 0.0, -236.0 } },
	{ { -5.0, 0.0, -239.0 },{ -3.0, 0.0, -239.0 },{ -0.25, 0.0, -239.0 },{ 0.25, 0.0, -239.0 },{ 3.0, 0.0, -239.0 },{ 5.0, 0.0, -239.0 } },
	{ { -5.0, 0.0, -242.0 },{ -3.0, 0.0, -242.0 },{ -0.25, 0.0, -242.0 },{ 0.25, 0.0, -242.0 },{ 3.0, 0.0, -242.0 },{ 5.0, 0.0, -242.0 } },
	{ { -5.0, 0.0, -245.0 },{ -3.0, 0.0, -245.0 },{ -0.25, 0.0, -245.0 },{ 0.25, 0.0, -245.0 },{ 3.0, 0.0, -245.0 },{ 5.0, 0.0, -245.0 } },
};

static float xvalues[6] = { -5.0,-3.0,-0.25,0.25,3.0,5.0 };
static float zvalues[33] = { 25.0,22.0,19.0,16.0,13.0,10.0,7.0,4.0,1.0,-2.0,-5.0,-8.0,-11.0,-14.0,-17.0,-20.0,-23.0,-26.0,-29.0,-32.0,-35.0,-38.0,-41.0,-44.0,-47.0,-50.0,-53.0,-56.0,-59.0,-62.0,-65.0 };

// Control points for the texture coordinates Bezier surface.
static float texturePoints[2][2][2] =
{
	{ { 0.0, 0.0 },{ 0.0, 1.0 } },
	{ { 1.0, 0.0 },{ 1.0, 1.0 } }
};


static unsigned int base; // Base index for display lists.
static GLUquadricObj *qobj; // Pointer to GLU quadric object.
static long font = (long)GLUT_BITMAP_8_BY_13;

static float zmove = 13.0;
static float xmove = 0.0;
static float ymove = 0.5;
static int x1move = -3.0, x2move = 3.0;
static int x3move = -3.0, x4move = 3.0;

static float angle = 0.0;
static float xVal = 0, zVal = 0;
static float radius = 0.5;

static int isAnimate = 0; // Animated?
static int animationPeriod = 100; // Time interval between frames.

static unsigned int texture[10]; // Array of texture indices.
static int filter = 18; // Filter id.
static int boxturn = 0;
/**************************************** End globals ****************************************/

//// Routine to draw a stroke character string.

void writeBitmapString(void *font, char *string)
{
	char *c;

	for (c = string; *c != '\0'; c++) glutBitmapCharacter(font, *c);
}


// Timer function.
void animate(int value)
{
	if (isAnimate && zmove>-235.0)
	{
		if (zmove <= -35.0 && zmove >= -50.0) {

			ymove -= 0.1;
			//cout << "y1="<<ymove << '\n';
			zmove -= 1.5;
		}
		else if (zmove <= -50.0 && zmove > -62.0) {
			ymove += 0.1;
			//cout << "y2="<<ymove << '\n';
			zmove -= 1.0;
		}
		else if (zmove <= -70.0 && zmove > -73.0) {
			ymove += 0.3;
			//cout << "y1=" << ymove << '\n';
			zmove -= 1.0;
		}
		else if (zmove <= -73.0 && zmove > -74.5) {
			ymove -= 0.3;
			float zmove1 = -76.0 - 1.0;
			zmove -= 1.0;
			//cout << "y2=" << ymove << '\n';
		}
		else if (zmove <= -74.5 && zmove > -170.0) {
			ymove = 0.5;
			float zmove1 = -75.0 - 1.0;
			zmove -= 1.0;
			//cout << "y2=" << ymove << '\n';
		}		
		else if (zmove <= -170.0 && zmove > -180.0) {
			ymove += 0.2;
			//cout << "y1="<<ymove << '\n';
			zmove -= 1.0;
		}
		else if (zmove <= -180.0 && zmove > -200.0) {
			float ymove1 = 1.7-0.1;
			ymove -= 0.1;
		//	cout << "y2=" << ymove << '\n';
			zmove -= 1.5;
		}
		else if (zmove <= -200.0) {
			//float ymove1 = 0.5-0.1;
			ymove = 0.5;
			//cout << "y2=" << ymove << '\n';
			zmove -= 1.0;
		}
		else
		zmove -= 1.0;
	

		/*********************collision***********************/
		if (xmove == 0.0 && ymove == 0.5 && zmove == -14.0)
		{
			isAnimate = 0;

			glColor3f(1.0, 0.0, 0.0);

			glRasterPos3f(0.0, 1.0, zmove);
			writeBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, "GAME OVER Try again");

		}
		if (xmove >= x1move - 0.4 && xmove <= x1move + 0.4 && ymove == 0.5 && zmove <= -4.0 && zmove >= -5.0)
		{
			isAnimate = 0;
		}

		if (xmove >= x1move - 0.4 && xmove <= x1move + 0.4 && ymove == 0.5 && zmove <= -159.0 && zmove >= -160.0)
		{
			isAnimate = 0;
		}


		if (xmove <= -x3move + 0.4 && xmove >= -x3move - 0.4 && ymove == 0.5 && zmove <= -169.0 && zmove >= -170.0)
		{
			isAnimate = 0;
		}
		if (xmove >= -0.4 && xmove <= 0.4 && ymove == 0.5 && zmove <= 1.0 &&zmove >= 0.0)
		{
			isAnimate = 0;
		}
		if (xmove >= -2.6 && xmove <= -3.4 && ymove == 0.5 && zmove <= -154.0 &&zmove >= -155.0)
		{
			isAnimate = 0;
		}
		if (xmove <= 3.4 && xmove >= 2.6 && ymove == 0.5 && zmove <= -154.0 &&zmove >= -155.0)
		{
			isAnimate = 0;
		}
		//d
		if (xmove >= -0.1 && xmove <= 0.5 && ymove <= 0.5 && zmove <= -108.0 && zmove >= -109.0)
		{
			isAnimate = 0;
		}

		if (xmove <= 0.9 && xmove >= 0.1 && ymove == 0.5 && zmove <= -108.0 && zmove >= -109.0)
		{
			isAnimate = 0;
		}

		if (xmove >= -1.1 &&xmove <= -1.9 && ymove == 0.5 && zmove <= -109.0 && zmove >= -110.0)
		{
			isAnimate = 0;
		}
		else if (xmove <= 1.9 && xmove >= 1.1 && ymove == 0.5 && zmove <= -109.0 && zmove >= -110.0)
		{
			isAnimate = 0;
		}
		else if (xmove >= -3.6 && xmove <= -4.4 && ymove == 0.5 && zmove <= -109.0 && zmove >= -110.0)
		{
			isAnimate = 0;
		}
		else if (xmove >= -3.6 && xmove <= -4.4 && ymove == 0.5 &&  zmove <= -219.0 && zmove >= -220.0)
		{
			isAnimate = 0;
		}
		else if (xmove >= -2.6 && xmove <= -3.4 && ymove == 0.5 && zmove <= -219.0 && zmove >= -220.0)
		{
			isAnimate = 0;
		}
		//d
		else if (xmove >= -2.4 && xmove <= -1.6 && ymove == 0.5 &&
			zmove <= -219.0 && zmove >= -220.0)
		{
			isAnimate = 0;
		}
		else if (xmove >= 0.6 && xmove <= 1.4 && ymove == 0.5 &&zmove <= -219.0 && zmove >= -220.0)
		{
			isAnimate = 0;
		}
		else if (xmove >= 1.6 && xmove <= 2.4 && ymove == 0.5 && zmove <= -219.0 && zmove >= -220.0)
		{
			isAnimate = 0;
		}
		else if (xmove >= 2.6 && xmove <= 3.4 && ymove == 0.5 && zmove <= -219.0 && zmove >= -220.0)
		{
			isAnimate = 0;
		}

		else if (xmove >= 3.6 && xmove <= 4.4 && ymove == 0.5 && zmove <= -219.0 && zmove >= -220.0)
		{
			isAnimate = 0;
		}
		else if (xmove <= -4.4 && xmove >= -3.6 && ymove == 0.5 &&  zmove <= -9.0 && zmove >= -10.0)
		{
			isAnimate = 0;
		}
		else if (xmove >= 3.6 && xmove <= 4.4 && ymove == 0.5 &&  zmove <= -9.0 && zmove >= -10.0)
		{
			isAnimate = 0;
		}
		else if (xmove <= -4.4 && xmove >= -3.6  && ymove == 0.5 && zmove <= -85.0 && zmove >= -86.0)
		{
			isAnimate = 0;
		}
		else if (xmove >= 3.6 && xmove <= 4.4 && ymove == 0.5 && zmove <= -85.0 && zmove >= -86.0)
		{
			isAnimate = 0;
		}
		else if (xmove >= 3.6 && xmove <= 4.4&& ymove == 0.5 && zmove <= -71.0 && zmove >= -72.0)
		{
			isAnimate = 0;
		}

		else if (xmove <= -4.4 && xmove >= -3.6  && ymove == 0.5 &&  zmove <= -71.0 && zmove >= -72.0)
		{
			isAnimate = 0;
		}

		if (boxturn == 0) {
			x1move += 1.0;
			x3move += 1.0;
			if (x1move == 3 && x3move == 3) boxturn = 1;
		}

		if (boxturn == 1) {
			x1move -= 1.0;
			x3move -= 1.0;
			if (x1move == -3 && x3move==-3) boxturn = 0;
		}
		glutPostRedisplay();
		glutTimerFunc(animationPeriod, animate, 1);
	}

}

// Load external textures.
void loadTextures()
{
	// Local storage for bmp image data.
	imageFile *image[25];

	// Load the textures.
	image[0] = getBMP("../../Textures/start.bmp");
	image[1] = getBMP("../../Textures/track.bmp");
	image[2] = getBMP("../../Textures/cube.bmp");

//	image[3] = getBMP("../../Textures/back.bmp");

	// Bind start checkered image to texture[0].
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[0]->width, image[0]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[0]->data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Bind tarck image to texture[1].
	glBindTexture(GL_TEXTURE_2D, texture[1]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Bind track image to texture[2].
	glBindTexture(GL_TEXTURE_2D, texture[2]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Bind track image to texture[3].
	glBindTexture(GL_TEXTURE_2D, texture[3]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Bind track image to texture[4].
	glBindTexture(GL_TEXTURE_2D, texture[4]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Bind track image to texture[5].
	glBindTexture(GL_TEXTURE_2D, texture[5]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Bind track image to texture[6].
	glBindTexture(GL_TEXTURE_2D, texture[6]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Bind ramp1 image to texture[7].
	glBindTexture(GL_TEXTURE_2D, texture[7]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[2]->width, image[2]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[2]->data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	// Bind background1 image to texture[8].
	glBindTexture(GL_TEXTURE_2D, texture[8]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Bind background1 image to texture[9].
	glBindTexture(GL_TEXTURE_2D, texture[9]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Bind background1 image to texture[10].
	glBindTexture(GL_TEXTURE_2D, texture[10]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Bind background1 image to texture[11].
	glBindTexture(GL_TEXTURE_2D, texture[11]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Bind background1 image to texture[12].
	glBindTexture(GL_TEXTURE_2D, texture[12]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Bind background1 image to texture[13].
	glBindTexture(GL_TEXTURE_2D, texture[13]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Bind background1 image to texture[14].
	glBindTexture(GL_TEXTURE_2D, texture[14]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Bind background1 image to texture[15].
	glBindTexture(GL_TEXTURE_2D, texture[15]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Bind background1 image to texture[16].
	glBindTexture(GL_TEXTURE_2D, texture[16]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Bind background1 image to texture[17].
	glBindTexture(GL_TEXTURE_2D, texture[17]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[1]->width, image[1]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[1]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Bind background1 image to texture[18].
	glBindTexture(GL_TEXTURE_2D, texture[18]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[0]->width, image[0]->height, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, image[0]->data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//// Bind background1 image to texture[18].
	//glBindTexture(GL_TEXTURE_2D, texture[19]);
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image[3]->width, image[3]->height, 0,
	//	GL_RGBA, GL_UNSIGNED_BYTE, image[3]->data);
	//glGenerateMipmap(GL_TEXTURE_2D);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}
// Initialization routine.
void setup(void)
{
	glEnableClientState(GL_VERTEX_ARRAY);

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glEnable(GL_DEPTH_TEST); // Enable depth testing.
	glutTimerFunc(5, animate, 1);

	//texture
	glGenTextures(16, texture);
	loadTextures();

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	glMap2f(GL_MAP2_TEXTURE_COORD_2, 0, 1, 2, 2, 0, 1, 4, 2, texturePoints[0][0]);
	glEnable(GL_MAP2_TEXTURE_COORD_2);

	glEnable(GL_AUTO_NORMAL); // Enable automatic normal calculation.

	glEnable(GL_LIGHTING);
	// Light property vectors.
	float lightAmb[] = { 1.0, 0.0, 0.0, 1.0 };
	float lightDifAndSpec[] = { 1.0, 1.0, 1.0, 1.0 };
	float lightPos[] = { 0.0, 1.5, 3.0, 0.0 };
	float globAmb[] = { 0.2, 0.2, 0.2, 1.0 };
	// Light properties.
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmb);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDifAndSpec);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightDifAndSpec);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

	glEnable(GL_LIGHT0); // Enable particular light source.
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globAmb); // Global ambient light.
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE); // Enable two-sided lighting.
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE); // Enable local viewpoint.

}

// Drawing routine.
void drawScene(void)
{


	int i, j;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	//(GLdouble eyeX,GLdouble eyeY,GLdouble eyeZ,GLdouble centerX,GLdouble centerY,GLdouble centerZ,GLdouble upX,GLdouble upY,GLdouble upZ);
	gluLookAt(0.0, 3.0 + (ymove/2), 10 + zmove, 0.0, 0.0, zmove, 0.0, 1.0, 0.0);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	/********************************************Racing Ball*********************************************/

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	//cout << "x++"<<xmove<<"z++"<<zmove<<"\n";
	glTranslatef(xmove, ymove, zmove);
	glutSolidSphere(radius, 50, 50);
	glPopMatrix();		

	/*******************************************Obstacles*************************************************/

	glPushMatrix();
	glColor3f(0.0, 1.0, 0.0);
	glTranslatef(x1move, 0.5, -5);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0);
	glTranslatef(-x3move, 0.5, 5);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0);
	glTranslatef(0.0, 0.5, 0.0);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(x1move, 0.5, -165);
	glutSolidCube(1.0);
	glPopMatrix();


	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-x3move, 0.5, -170);
	glutSolidCube(1.0);
	glPopMatrix();


	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-0.25, 0.5, -14);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-3.0, 0.5, -155);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(3.0, 0.5, -155);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-0.25, 0.5, -155);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(0.25, 0.5, -155);
	glutSolidCube(1.0);
	glPopMatrix();
	/**************************************statue patch******************************************/
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-1.5, 0.5, -110);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(1.5, 0.5, -110);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(0.5, 0.5, -110);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-0.5, 0.5, -111);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-0.5, 0.5, -109);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-0.5, 1.5, -110);
	glutSolidCube(1.0);
	glPopMatrix();
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(0.5, 0.5, -111);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(0.5, 0.5, -109);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(0.5, 1.5, -110);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-0.0, 2.0, -110);
	glutSolidCube(0.5);
	glPopMatrix();


	
	/**************************************RAMP-1 patch******************************************/
	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Turn on OpenGL texturing.
	glEnable(GL_TEXTURE_2D);
	// Map the cube texture onto a rectangle along the xz-plane.
	glBindTexture(GL_TEXTURE_2D, texture[7]);

	// Draw a polygon with specified vertices.
	glBegin(GL_POLYGON);
	glTexCoord2f(0.0, 0.0); glVertex3f(-3.0, 0.0, -70.0);
	glTexCoord2f(1.0, 0.0); glVertex3f(3.0, 0.0, -70.0);
	glTexCoord2f(1.0, 1.0); glVertex3f(3.0, 1.0, -74.0);
	glTexCoord2f(0.0, 1.0); glVertex3f(-3.0, 1.0, -74.0);
	glEnd();
	glPopMatrix();

	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0);
	glTranslatef(-4.0, 0.5, -72.0);
	glutSolidCube(1.5);
	glPopMatrix();

	glPushMatrix();
	glColor3f(0.0, 0.0, 1.0);
	glTranslatef(4.0, 0.5, -72.0);
	glutSolidCube(1.5);
	glPopMatrix();

	glDisable(GL_TEXTURE_2D);

	// Turn on OpenGL texturing.
	glEnable(GL_TEXTURE_2D);

	/*glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glBegin(GL_POLYGON);
	glVertex3f(3.0, 0.0, -130.0);
	glVertex3f(5.0, 0.0, -130.0);
	glVertex3f(5.0, 1.0, -134.0);
	glVertex3f(3.0, 1.0, -134.0);


	glEnd();
	glPopMatrix();*/
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-3.0, 0.5, -132.0);
	glutSolidCube(1.5);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(0.0, 0.5, -132.0);
	glutSolidCube(1.5);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-1.5, 0.5, -132.0);
	glutSolidCube(1.5);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(1.5, 0.5, -132.0);
	glutSolidCube(1.5);
	glPopMatrix();

	/**************************************half arch patch******************************************/
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-4.0, 0.5, -140);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(4.0, 0.5, -160.0);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-4.0, 1.5, -140.0);
	glutSolidCube(1.0);
	glPopMatrix();


	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-4.0, 2.5, -140.0);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(4.0, 1.5, -160.0);
	glutSolidCube(1.0);
	glPopMatrix();


	for (float i = -4.0; i <5; i++) {
		glPushMatrix();
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(i, 2.5, -160.0);
		glutSolidCube(1.0);
		glPopMatrix();
	}

	for (float i = -4.0; i < -1.0; i++) {
		glPushMatrix();
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(i, 2.5, -140.0);
		glutSolidCube(1.0);
		glPopMatrix();
	}


	for (float i = -4.0; i < -1.0; i++) {
		glPushMatrix();
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(i, 0.5, -220.0);
		glutSolidCube(1.0);
		glPopMatrix();
	}


	for (float i = 1.0; i < 5.0; i++) {
		glPushMatrix();
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(i, 0.5, -220.0);
		glutSolidCube(1.0);
		glPopMatrix();
	}

	/**************************************arch patch******************************************/
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-4.0, 0.5, -10.0);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(4.0, 0.5, -10.0);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-4.0, 1.5, -10.0);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(4.0, 1.5, -10.0);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-4.0, 2.5, -10.0);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(4.0, 2.5, -10.0);
	glutSolidCube(1.0);
	glPopMatrix();



	for (float i = -4.0; i < -1.0; i++) {
		glPushMatrix();
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(i, 3.5, -10.0);
		glutSolidCube(1.0);
		glPopMatrix();
	}

	for (float i = 2.0; i < 5.0; i++) {
		glPushMatrix();
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(i, 3.5, -10.0);
		glutSolidCube(1.0);
		glPopMatrix();
	}
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-4.0, 0.5, -86);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(4.0, 0.5, -86.0);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-4.0, 1.5, -86.0);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(4.0, 1.5, -86.0);
	glutSolidCube(1.0);
	glPopMatrix();
	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(-4.0, 2.5, -86.0);
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glColor3f(1.0, 0.0, 0.0);
	glTranslatef(4.0, 2.5, -86.0);
	glutSolidCube(1.0);
	glPopMatrix();

	for (float i = -4.0; i < 5.0; i++) {
		glPushMatrix();
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(i, 3.5, -86.0);
		glutSolidCube(1.0);
		glPopMatrix();
	}
	glDisable(GL_TEXTURE_2D);
	/**************************************sink holes******************************************/
	//glPushMatrix();
	//glColor3f(1.0, 1.0, 0.0);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	//// Draw a polygon with specified vertices.
	//glBegin(GL_POLYGON);
	//glVertex3f(-3.0, 0.0, -205.0);
	//glVertex3f(3.0, 0.0, -205.0);
	//glVertex3f(3.0, 0.0, -210.0);
	//glVertex3f(-3.0, 0.0, -210.0);
	//glEnd();
	//glPopMatrix();
	
	/**************************************Background-1 Patch******************************************/
	//// Turn on OpenGL texturing.
	//glEnable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D, texture[8]);
	//glBegin(GL_POLYGON);
	//glTexCoord2f(0.0, 0.0); glVertex3f(-100.0, 0.0, 25.0);
	//glTexCoord2f(1.0, 0.0); glVertex3f(-5.0, 0.0, 25.0);
	//glTexCoord2f(1.0, 1.0); glVertex3f(-100.0, 0.0, -100.0);
	//glTexCoord2f(0.0, 1.0); glVertex3f(100.0, 0.0, -100.0);
	//glEnd();
	/**************************************Starting Track Patch******************************************/

	// Turn on OpenGL texturing.
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	float matAmbAndDif1[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif2[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif1);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif2);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, start[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);


	/**************************************Track Patch-1*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);


	float matAmbAndDif3[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif4[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec1[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine1[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif3);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif4);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec1);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine1);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlPoints[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[1]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);


	/**************************************Track Patch-2*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif5[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif6[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec2[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine2[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif5);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif6);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec2);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine2);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlPoints_1[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[2]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track Patch-3*************************************************/

	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif7[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif8[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec3[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine3[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif7);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif8);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec3);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine3);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlPoints_2[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[3]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track Patch-4*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif9[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif10[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec4[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine4[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif9);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif10);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec4);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine4);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlPoints_3[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[4]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track Patch-5*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif11[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif12[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec5[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine5[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif11);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif12);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec5);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine5);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlPoints_4[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[5]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track Patch-6 with ramp*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif13[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif14[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec6[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine6[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif13);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif14);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec6);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine6);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlPoints_5[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[6]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track Patch-7*************************************************/

	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif15[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif16[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec7[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine7[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif15);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif16);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec7);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine7);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlPoints_6[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[8]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	/**************************************Track Patch-8statue*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif17[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif18[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec8[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine8[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif17);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif18);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec8);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine8);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlPoints_7[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[9]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	/**************************************Track Patch-9statue*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif19[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif20[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec9[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine9[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif19);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif20);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec8);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine8);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlPoints_8[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[10]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track Patch-10*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDifa[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDifb[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec17[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine17[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDifa);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDifb);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec17);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine17);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlPoints_9[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[11]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track Patch-11*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif21[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif22[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec10[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine10[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif21);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif22);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec10);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine10);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlpoints_10[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[12]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track Patch-12*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif23[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif24[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec11[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine11[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif23);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif24);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec11);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine11);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlpoints_11[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[13]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track Patch-13*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif25[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif26[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec12[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine12[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif25);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif26);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec12);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine12);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlpoints_12[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[14]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	/**************************************Track Patch-14*************************************************/
	
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif27[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif28[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec13[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine13[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif27);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif28);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec13);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine13);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlpoints_13[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[15]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track  Patch-15*************************************************/
	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif29[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif30[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec14[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine14[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif29);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif30);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec14);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine14);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlpoints_14[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[16]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track finish Patch-16*************************************************/

	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif31[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif32[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec15[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine15[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif31);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif32);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec15);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine15);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlpoints_15[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[17]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	/**************************************Track Patch-17*************************************************/

	glEnable(GL_TEXTURE_2D);
	// Draw the control polyhedron in light gray.
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.7, 0.7, 0.7);

	float matAmbAndDif33[] = { 0.9, 0.0, 0.0, 1.0 };
	float matAmbAndDif34[] = { 1.0, 0.1, 0.1, 1.0 };
	float matSpec16[] = { 1.0, 1.0, 1.0, 1.0 };
	float matShine16[] = { 50.0 };
	// Material properties.
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif33);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbAndDif34);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpec16);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, matShine16);
	// Specify and enable the Bezier surface.
	glMap2f(GL_MAP2_VERTEX_3, 0, 1, 3, 6, 0, 1, 18, 6, controlpoints_16[0][0]);
	glEnable(GL_MAP2_VERTEX_3);

	// Draw the Bezier surface using a mesh approximation.
	glColor3f(1.0, 0.0, 0.0);
	glEnable(GL_AUTO_NORMAL);
	glMapGrid2f(20, 0.0, 1.0, 20, 0.0, 1.0);

	// Map the checkered texture onto the blade Bezier surface.
	glBindTexture(GL_TEXTURE_2D, texture[18]);
	glEvalMesh2(GL_FILL, 0, 20, 0, 20);
	glDisable(GL_TEXTURE_2D);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Specify how texture values combine with current surface color values.
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	glutSwapBuffers();
}


// OpenGL window reshape routine.
void resize(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (float)w / (float)h, 1.0, 50.0);
	glMatrixMode(GL_MODELVIEW);
}

// Keyboard input processing routine.
void keyInput(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
		break;
	case 'w':
	//	cout << ""<<xmove<<"&"<<zmove<<"\n";
		if (isAnimate) isAnimate = 0;
		else
		{
			isAnimate = 1;
			animate(1);
		}
		break;
	case 'd':
		if (xmove < 4.0) xmove += 1.0;
		//glutPostRedisplay();
		break;
	case 'a':
		if (xmove > -4.0) xmove -= 1.0;
		//glutPostRedisplay();
		break;
		
	case 'r':
		zmove = 13.0;
		xmove = 0.0;
		ymove = 0.5;
		x1move = -3.0, x2move = 3.0;
		x3move = -3.0, x4move = 3.0;
		isAnimate = 0;
		//Xangle = 15.0, Yangle = 0.0, Zangle = 0.0;
		glutPostRedisplay();
		break;
	default:
		break;
	}
}

// Routine to output interaction instructions to the C++ window.
void printInteraction(void)
{
	std::cout << "Interaction:" << std::endl;
	std::cout << "Press w to start/ continuously press on w to increase acceleration." << std::endl
		<< "Press a to move left." << std::endl
		<< "Press d to move right." << std::endl
		<< "Press r to move reset." << std::endl;
}

// Main routine.
int main(int argc, char **argv)
{
	printInteraction();
	glutInit(&argc, argv);

	glutInitContextVersion(4, 3);
	glutInitContextProfile(GLUT_COMPATIBILITY_PROFILE);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(750, 750);
	glutInitWindowPosition(500, 100);
	glutCreateWindow("Final Project CGA.cpp");
	glutDisplayFunc(drawScene);
	glutReshapeFunc(resize);
	glutKeyboardFunc(keyInput);

	glewExperimental = GL_TRUE;
	glewInit();

	setup();

	glutMainLoop();
}
