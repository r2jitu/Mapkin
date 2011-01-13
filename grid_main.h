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

#ifndef GRID_MAIN_H
#define GRID_MAIN_H

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
#define ENCODER_CLICKS_PER_CM 500.0f
#define TOP_SPEED_CM_PER_S 30.0f
#define KINECT_ATTACHED 0
#define SIM_DELAY .1f       // lower is faster

#endif
