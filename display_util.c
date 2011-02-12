/**
 * Handles displaying the grid as explored so far.  The grid is displayed
 * as an image, which is redrawn constantly by openGL.  However, the image
 * must be updated to match the array representation of the world, and
 * functions to handle this are found in this file.
 *
 * @author Alex Zirbel
 */

#include "mapkin.h"

/**
 * Wingtips are the points that make the robot position look like an arrow rather
 * than a dot when displayed.  Wingtips extend backward from the robot in any
 * of 8 positions given the current angle.
 */
void getWingtips(int i, int j, int* wingtips, pose2D* cur_pos)
{
    if(cur_pos->theta < PI/8 || cur_pos->theta >= 15*PI/8)
    {
        wingtips[0] = i+1;
        wingtips[1] = j+1;
        wingtips[2] = i+2;
        wingtips[3] = j+2;
        wingtips[4] = i+1;
        wingtips[5] = j-1;
        wingtips[6] = i+2;
        wingtips[7] = j-2;
    }
    else if(cur_pos->theta < 3*PI/8)
    {
        wingtips[0] = i;
        wingtips[1] = j+1;
        wingtips[2] = i;
        wingtips[3] = j+2;
        wingtips[4] = i+1;
        wingtips[5] = j;
        wingtips[6] = i+2;
        wingtips[7] = j;
    }
    else if(cur_pos->theta < 5*PI/8)
    {
        wingtips[0] = i-1;
        wingtips[1] = j+1;
        wingtips[2] = i-2;
        wingtips[3] = j+2;
        wingtips[4] = i+1;
        wingtips[5] = j+1;
        wingtips[6] = i+2;
        wingtips[7] = j+2;
    }
    else if(cur_pos->theta < 7*PI/8)
    {
        wingtips[0] = i-1;
        wingtips[1] = j;
        wingtips[2] = i-2;
        wingtips[3] = j;
        wingtips[4] = i;
        wingtips[5] = j+1;
        wingtips[6] = i;
        wingtips[7] = j+2;
    }
    else if(cur_pos->theta < 9*PI/8)
    {
        wingtips[0] = i-1;
        wingtips[1] = j-1;
        wingtips[2] = i-2;
        wingtips[3] = j-2;
        wingtips[4] = i-1;
        wingtips[5] = j+1;
        wingtips[6] = i-2;
        wingtips[7] = j+2;
    }
    else if(cur_pos->theta < 11*PI/8)
    {
        wingtips[0] = i-1;
        wingtips[1] = j;
        wingtips[2] = i-2;
        wingtips[3] = j;
        wingtips[4] = i;
        wingtips[5] = j-1;
        wingtips[6] = i;
        wingtips[7] = j-2;
    }
    else if(cur_pos->theta < 13*PI/8)
    {
        wingtips[0] = i-1;
        wingtips[1] = j-1;
        wingtips[2] = i-2;
        wingtips[3] = j-2;
        wingtips[4] = i+1;
        wingtips[5] = j-1;
        wingtips[6] = i+2;
        wingtips[7] = j-2;
    }
    else
    {
        wingtips[0] = i;
        wingtips[1] = j-1;
        wingtips[2] = i;
        wingtips[3] = j-2;
        wingtips[4] = i+1;
        wingtips[5] = j;
        wingtips[6] = i+2;
        wingtips[7] = j;
    }
}

int withinBounds(int x, int y)
{
    if(x < 0 || x >= GRIDSIZE || y < 0 || y >= GRIDSIZE)
        return 0;

    return 1;
}
