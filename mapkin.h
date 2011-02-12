/**
 * This header describes grid_main.h and grid_main.c.
 * 
 * Most of the code in these files was written by:
 * Alex Zirbel
 * Jitu Das
 * Working with the Mapkin project for autonomous mapping in Carnegie Mellon's
 * build18.
 *
 * The openGL components as well as Kinect drivers come from the openKinect
 * project, and the licensing information for this starter code is below:
 *
 * ----------------------------------------------------------------------------
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * Andrew Miller <amiller@dappervision.com>
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#ifndef MAPKIN_H
#define MAPKIN_H

// Special modes to enable different ways of running the program
#define KINECT_SIM 1
#define ENCODERS_SIM 1
#define FULL_DISPLAY 1
#define DEMO_MODE 0

#include "libfreenect.h"
#include "libfreenect_sync.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#define RCW_TO_IDX(R, C, W) (R*W + C)
#define GRIDSIZE 412
#define CELL_WIDTH 5
#define ROBOT_RADIUS 15.0f // in centimeters. This is a guess
#define PI 3.141592f
#define ENCODER_CLICKS_PER_CM 6562.0f
#define TOP_SPEED_CM_PER_S 30.0f
#define SIM_DELAY .1f       // lower is faster
#define MAX_ANGLE .392699   // Pi / 8. Max angle of kinect spread on one side.

/* Colors to be displayed on the image. */
#define EMPTY 0
#define SEEN 1
#define CURRENT_READING 2
#define WALL 3
#define BLOCKED_EMPTY 4
#define BLOCKED_SEEN 5
#define VISITED 6

/* A pose2D is the coordinate-frame representation of a point.  It is
 * represented with as much information as possible.  X is measured forward
 * from the robot's coordinate frame, y to the left, and theta is
 * counterclockwise from x to y. */
typedef struct pose2D {
    float x, y, theta;
} pose2D;

/* A grid point is the array location of a point, as stored in the image to be
 * displayed.  This means that x is measured horizontally and y vertically,
 * from the top left corner of the array. */
typedef struct gridPoint {
    int x, y;
} gridPoint;

/* Useful extra utilities */
#define MIN(a,b) ((a)>(b)?(b):(a))
#define MAX(a,b) ((a)<(b)?(b):(a))
#define XYtoIDX(i, j) (i*GRIDSIZE + j)

GLfloat* multiplyMatrix(GLfloat* mat1, GLfloat* mat2,
        int m1w, int m1h, int m2w);

#endif
