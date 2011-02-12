/**
 * To help with the intelligence and path planning components of Mapkin.
 *
 * Contains functions to decide where to go next, and to command the robot
 * to move.
 */

#include "mapkin.h"


void setGoal(int* grid, pose2D* cur_pos, gridPoint* goal_pos)
{
    float theta, m, m2, x, y, a;
    int i, j;

    int curX = (int)(-(cur_pos->x/CELL_WIDTH) + (GRIDSIZE/2) - .5);
    int curY = (int)(-(cur_pos->y/CELL_WIDTH) + (GRIDSIZE/2) - .5);


    findNearest(grid, EMPTY, &curX, &curY);

    goal_pos->x = curX;
    goal_pos->y = curY;


    /*for(theta=0; theta<2*PI; theta += .01)
    {
        // Calculate the line in point-slope form
        if(fabs(cos(theta)) < 0.0005)
            m = 10000000;
        else
            m = sin(theta)/cos(theta);

        if(fabs(sin(theta)) < 0.0005)
            m2 = 10000000;
        else
            m2 = cos(theta)/sin(theta);

        // determine quadrant
        int xSign = 1;
        int ySign = 1;
        if(theta < PI && theta >= PI / 2)
            xSign = -1;
        else if(theta < 3*PI / 2 && theta >= PI)
            xSign = ySign = -1;
        else if(theta < 2*PI && theta >= 3*PI/2)
            ySign = -1;

        for(a=0; ; a+=.3)
        {
            int gridX, gridY;

            // Use x if the line is more horizontal
            if(abs(m) <= 1)
            {
                x = xSign * a + goal_pos->x;
                y = m*(x - goal_pos->x) + goal_pos->y;
            }
            else
            {
                y = ySign * a + goal_pos->y;
                x = m2*(y - goal_pos->y) + goal_pos->x;
            }

            gridX = (int)x;  //(-(x/CELL_WIDTH) + (GRIDSIZE/2) - .5);
            gridY = (int)y;  //(-(y/CELL_WIDTH) + (GRIDSIZE/2) - .5);

            if(gridX < 0 || gridX >= GRIDSIZE || gridY < 0 || gridY >= GRIDSIZE)
                break;

            int pointStatus = grid[XYtoIDX(gridX,gridY)];
            if(pointStatus == SEEN || pointStatus == VISITED || pointStatus == EMPTY
                || pointStatus == BLOCKED_SEEN || pointStatus == BLOCKED_EMPTY)
            //if(grid[gridX][gridY] == SEEN || grid[gridX][gridY] == VISITED
            //      || grid[gridX][gridY] == EMPTY
            //      || grid[gridX][gridY] == BLOCKED_SEEN
            //      || grid[gridX][gridY] == BLOCKED_EMPTY)
            {
                // previously set approach points here
            }
        }
    }*/
    //printf("Set goal to (%d,%d)\n", goal_pos->x, goal_pos->y);
}
