/**
 * Contains functions to help with simulation, including loading the simulated
 * maze and generating depths based on that simulated setup.
 */

#include "mapkin.h"

/**
 * Fills in an array of integer depths (assumed to be 640 integers, for a
 * Kinect) with some generated sequence of numbers.
 */
void generateFixedDepths(int* depths)
{
    int i;
    for(i=0; i<640; i++)
    {
        depths[i] = 240;
    }
}

/**
 * Reads the depths from the sim maze
 */
void generateDepths(int* maze, pose2D* cur_pos, int* depths)
{
    int i;
    float a, x, y, m, m2;
    float theta;

    for(i=0; i<640; i++)
    {
        theta = (((float)i)-320) / 320 * MAX_ANGLE;
        theta += cur_pos->theta;
        putAngleInBounds(&theta);

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
        {
            xSign = -1;
        }
        else if(theta < 3*PI / 2 && theta >= PI)
        {
            xSign = ySign = -1;
        }
        else if(theta < 2*PI && theta >= 3*PI/2)
        {
            ySign = -1;
        }

        for(a=0; ; a+=.3)
        {
            int gridX, gridY;

            // Use x if the line is more horizontal
            if(abs(m) <= 1)
            {
                x = xSign * a + cur_pos->x;
                y = m*(x - cur_pos->x) + cur_pos->y;
            }
            else
            {
                y = ySign * a + cur_pos->y;
                x = m2*(y - cur_pos->y) + cur_pos->x;
            }

            gridX = (-(x/CELL_WIDTH) + (GRIDSIZE/2) - .5);
            gridY = (-(y/CELL_WIDTH) + (GRIDSIZE/2) - .5);

            if(gridX < 0 || gridX >= GRIDSIZE || gridY < 0 || gridY >= GRIDSIZE)
            {
                depths[i] = -1;
                break;
            }

            if(maze[XYtoIDX(gridX, gridY)] == 1)
            {
                depths[i] = sqrt(((x-cur_pos->x)*(x-cur_pos->x)) + ((y-cur_pos->y)*(y-cur_pos->y)));
                break;
            }
        }
    }
}

/**
 * Draws a sample maze for simulation runs.
 * TODO: this doesn't work. Why?
 */
/*void loadMaze(int* maze)
{
    int i, j, a;
    for(i=0; i<GRIDSIZE; i++)
    {
        for(j=0; j<GRIDSIZE; j++)
        {
            maze[XYtoIDX(i,j)] = 0;
        }
    }

    // Draw borders
    for(a=0; a<GRIDSIZE; a++)
    {
        maze[XYtoIDX(0,a)] = 1;
        maze[XYtoIDX(GRIDSIZE-1,a)] = 1;
        maze[XYtoIDX(a,0)] = 1;
        maze[XYtoIDX(a, GRIDSIZE-1)] = 1;
    }

    // Draw some walls
    for(a=0; a<80; a++)
    {
        // horizontal
        maze[XYtoIDX(80,a)] = 1;
        maze[XYtoIDX(80,a+160)] = 1;
        maze[XYtoIDX(80,a+240)] = 1;
        maze[XYtoIDX(160,a+240)] = 1;
        maze[XYtoIDX(240,a)] = 1;
        maze[XYtoIDX(320,a+80)] = 1;
        maze[XYtoIDX(240,a+240)] = 1;
        maze[XYtoIDX(320,a)] = 1;

        maze[80][a] = 1;
        maze[80][a+160] = 1;
        maze[80][a+240] = 1;
        maze[160][a+240] = 1;
        maze[240][a] = 1;
        maze[320][a+80] = 1;
        maze[240][a+240] = 1;
        maze[320][a] = 1;

        // vertical
        maze[(a+80)*GRIDSIZE + 80] = 1;
        maze[(a)*GRIDSIZE + 160] = 1;
        maze[XYtoIDX(a+160, 240)] = 1;
        maze[XYtoIDX(a+320, 240)] = 1;
        maze[XYtoIDX(a+80, 320)] = 1;
        maze[XYtoIDX(a+240, 320)] = 1;

        maze[XYtoIDX(a+80,80)] = 1;
        maze[XYtoIDX(a,160)] = 1;
        maze[XYtoIDX(a+160,240)] = 1;
        maze[XYtoIDX(a+320,240)] = 1;
        maze[XYtoIDX(a+80,320)] = 1;
        maze[XYtoIDX(a+240,320)] = 1;

        maze[a+80][80] = 1;
        maze[a][160] = 1;
        maze[a+160][240] = 1;
        maze[a+320][240] = 1;
        maze[a+80][320] = 1;
        maze[a+240][320] = 1;
    }
}*/
