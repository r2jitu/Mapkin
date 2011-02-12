/** 
 * A utilities file for dealing with coordinate frame representation and
 * conversion.  Provides two definitions for points:
 *
 * pose2D describes a point and angle relative to a reference frame where
 *   x points forward, y points left, and theta is measured counterclockwise
 *   from x to y.
 * gridPoint describes the array location of a point in the image to be
 *   displayed.
 *
 * This file contains utilities for converting a pose to a grid point,
 * putting a pose in the coordinate frame of another pose, and so on.
 * 
 * @author Alex Zirbel
 */

#include "mapkin.h"



/**
 * Returns an angle between 0 and 2 PI.
 */
void putAngleInBounds(float * theta)
{
    while(*theta < 0)
        *theta += 2 * PI;
    while(*theta > 2 * PI)
        *theta -= 2 * PI;
}

/**
 * Finds the nearest grid cell with color equal to color specified, and returns
 * it in the same x and y that specify the original point to search from.
 *
 * A Breadth-First search.
 * Color -1 means look for an adjacent point.
 */
void findNearest(int* grid, int color, int *x, int *y)
{
    // My code: 1 means to search in this iteration.
    // 2 Means to search in the next iteration.
    // 3 Means already searched, now dead.
    int i, j, a, b;
    int to_search[GRIDSIZE][GRIDSIZE] = {{0}};
    to_search[*x][*y] = 1;

    while(moreToSearch(to_search))
    {
        for(i=0; i<GRIDSIZE; i++)
        {
            for(j=0; j<GRIDSIZE; j++)
            {
                if(to_search[i][j] == 1)
                {
                    // Check if this matches the color

                    // TODO: changed this
                    if(grid[XYtoIDX(i,j)] == color)
                    {
                        *x = i;
                        *y = j;
                        return;
                    }

                    // Label the cells to be searched next iteration
                    for(a=MAX(0, i-1); a<MIN(GRIDSIZE, i+2); a++)
                    {
                        for(b=MAX(0, j-1); b<MIN(GRIDSIZE, j+2); b++)
                        {
                            if(to_search[a][b] == 0)
                                to_search[a][b] = 2;
                        }
                    }

                    // Label this node as checked
                    to_search[i][j] = 3;
                }
            }
        }

        // Prepare for the next iteration
        for(i=0; i<GRIDSIZE; i++)
        {
            for(j=0; j<GRIDSIZE; j++)
            {
                if(to_search[i][j] == 2)
                    to_search[i][j] = 1;
            }
        }

    }

    // Error code for not found
    *x = -1;
    *y = -1;
}

/* Returns true if there are still 1's in the grid (assumed standard size)*/
int moreToSearch(int* to_search)
{
    int i, j;
    for(i=0; i<GRIDSIZE*GRIDSIZE; i++)
    {
        if(to_search[i] == 1)
            return 1;
    }
    return 0;
}

/**
 * Prints out the x, y, theta of a pose2D, for debugging.
 */
void displayPose2D(pose2D *pose)
{
    printf("[%f, %f, %f]", pose->x, pose->y, pose->theta);
}

/**
 * Assuming the point is in the right frame (relative to the grid, with x upward
 * in the center of the grid, y left, theta counterclockwise from x
 *
 */
void drawReadingOnGrid(int* grid, pose2D *point)
{
    int i, j;

    int x = (-(point->x/CELL_WIDTH) + (GRIDSIZE/2) - .5);
    int y = (-(point->y/CELL_WIDTH) + (GRIDSIZE/2) - .5);
    // (GRIDSIZE/2) - (int)((point->x + .5)/CELL_WIDTH) - 1;
    //int y = (GRIDSIZE/2) - (int)((point->y + .5)/CELL_WIDTH) - 1;

    // These loops added because the netbook is too slow
    if(FULL_DISPLAY)
    {
        for(i=MAX(0, x-ROBOT_RADIUS); i<MIN(GRIDSIZE, x+ROBOT_RADIUS); i++)
        {
            for(j=MAX(0, y-ROBOT_RADIUS); j<MIN(GRIDSIZE, y+ROBOT_RADIUS); j++)
            {
                if(sqrt((x-i)*(x-i) + (y-j)*(y-j)) <= ROBOT_RADIUS)
                {
                    if(grid[XYtoIDX(i,j)] == EMPTY)
                        grid[XYtoIDX(i,j)] = BLOCKED_EMPTY;
                    else if(grid[XYtoIDX(i,j)] == SEEN)
                        grid[XYtoIDX(i,j)] = BLOCKED_SEEN;
                }
            }
        }
    }

    if(x >= 0 && x < GRIDSIZE && y >= 0 && y < GRIDSIZE)
        grid[XYtoIDX(x,y)] = CURRENT_READING;
}

/**
 * The current sensor reading spots on the grid (2) become old readings (1).
 */
void staleGrid(int* grid)
{
    int i, j;
    for(i=0; i<GRIDSIZE; i++)
    {
        for(j=0; j<GRIDSIZE; j++)
        {
            if(grid[XYtoIDX(i,j)] == CURRENT_READING)
                grid[XYtoIDX(i,j)] = WALL;
        }
    }
}

/**
 * Erases all readings from the grid. Note that origin and position are
 * never erased.
 */
void eraseGrid(int* grid)
{
    int i, j;
    for(i=0; i<GRIDSIZE; i++)
    {
        for(j=0; j<GRIDSIZE; j++)
        {
            grid[XYtoIDX(i,j)] = EMPTY;
        }
    }
}

/**
 * Converts the pose to convert into the world frame, assuming it is currently
 * in the "frame" frame.
 */
void convertToFrame(pose2D* to_convert, pose2D* frame)
{
    GLfloat transform[16] = {
        cos(frame->theta), -sin(frame->theta), frame->x,
        sin(frame->theta), cos(frame->theta), frame->y,
        0, 0, 1};
    GLfloat matrix_to_convert[3] = {to_convert->x, to_convert->y, 1};


    GLfloat* result = multiplyMatrix(transform, matrix_to_convert, 3, 3, 1);

    to_convert->x = result[0];
    to_convert->y = result[1];
    to_convert->theta = to_convert->theta + frame->theta;
    putAngleInBounds(&(to_convert->theta));
}
