/**
 * Authors: Alex Zirbel, Jitu Das, and the openKinect project. See
 * mapkin.h for details.
 */

#include "mapkin.h"
#include "serial_comm.h"

int window;
GLuint gl_depth_tex;
GLuint gl_rgb_tex;
int mx=-1,my=-1;        // Prevous mouse coordinates
int rotangles[2] = {0}; // Panning angles
float zoom = 1;         // zoom factor
int color = 1;          // Use the RGB texture or just draw it as color
int count = 0;
int die = 0;

extern int motor_l_speed;
extern int motor_r_speed;

// Function declarations
void displayGrid();
void set_curvature(float curvature, int speed);

int grid[GRIDSIZE][GRIDSIZE] = {{0}};   // Mark the places we have explored and seen a wall
int maze[GRIDSIZE][GRIDSIZE] = {{0}};   // For simulation, mark boundary walls.
char grid_image[GRIDSIZE][GRIDSIZE][3];

short *depth = 0;

/* Planning to use linked lists to store the kinect information from each
 * individual reading, then looping through all stored readings to display
 * a 3D point cloud. */
typedef struct llnode {
    struct llnode *next, *prev;
    void *data;
} llnode;

uint8_t *depth_mid;
uint16_t t_gamma[2048];
llnode *head = NULL, *tail = NULL;


pose2D *cur_pos = NULL;
gridPoint *goal_pos = NULL; // Stored in grid coordinates.

void llAddLast(void *data) {
    llnode *node = malloc(sizeof(llnode));
    node->data = data;

    if (head == NULL) head = node;
    if (tail != NULL) tail->next = node;
    node->prev = tail;
    node->next = NULL;
    tail = node;
}



/**
 * Draws a sample maze for simulation runs.
 */
void loadMaze()
{
    int i, j, a;
    for(i=0; i<GRIDSIZE; i++)
    {
        for(j=0; j<GRIDSIZE; j++)
        {
            maze[i][j] = 0;
        }
    }

    // Draw borders
    for(a=0; a<GRIDSIZE; a++)
    {
        maze[0][a] = 1;
        maze[GRIDSIZE-1][a] = 1;
        maze[a][0] = 1;
        maze[a][GRIDSIZE-1] = 1;
    }

    // Draw some walls
    for(a=0; a<80; a++)
    {
        // horizontal
        maze[80][a] = 1;
        maze[80][a+160] = 1;
        maze[80][a+240] = 1;
        maze[160][a+240] = 1;
        maze[240][a] = 1;
        maze[320][a+80] = 1;
        maze[240][a+240] = 1;
        maze[320][a] = 1;

        // vertical
        maze[a+80][80] = 1;
        maze[a][160] = 1;
        maze[a+160][240] = 1;
        maze[a+320][240] = 1;
        maze[a+80][320] = 1;
        maze[a+240][320] = 1;
    }
}

/**
 * Prints out the grid to the grid_image, which is constantly being displayed.
 * Responsible for setting all the map colors.
 */
void displayGrid()
{
    int i=0, j=0, a=0;
    int curX = (int)(-(cur_pos->x/CELL_WIDTH) + (GRIDSIZE/2) - .5);
    int curY = (int)(-(cur_pos->y/CELL_WIDTH) + (GRIDSIZE/2) - .5);

    //printf("Current position: [%f %f %f]\n", cur_pos->x, cur_pos->y, cur_pos->theta);
    //printf("Current speed: [%d %d]\n", motor_l_speed, motor_r_speed);

    for(i=0; i<GRIDSIZE; i++)
    {
        for(j=0; j<GRIDSIZE; j++)
        {
            // Old wall seen here
            if(grid[i][j] == WALL)
            {
                grid_image[i][j][0] = 255;
                grid_image[i][j][1] = 215;
                grid_image[i][j][2] = 0;
            }
            // Current sensor reading
            else if(grid[i][j] == CURRENT_READING)
            {
                grid_image[i][j][0] = 255;
                grid_image[i][j][1] = 0;
                grid_image[i][j][2] = 0;
            }
            // Inaccessible area
            else if(grid[i][j] == BLOCKED_SEEN)
            {
                grid_image[i][j][0] = 108;
                grid_image[i][j][1] = 0;
                grid_image[i][j][2] = 211;
            }
            // Inaccessible area
            else if(grid[i][j] == BLOCKED_EMPTY)
            {
                grid_image[i][j][0] = 65;
                grid_image[i][j][1] = 0;
                grid_image[i][j][2] = 134;
            }
            // Seen already
            else if(grid[i][j] == SEEN)
            {
                grid_image[i][j][0] = 0;
                grid_image[i][j][1] = 0;
                grid_image[i][j][2] = 200;
            }
            // Where the robot itself has been
            else if(grid[i][j] == VISITED)
            {
                grid_image[i][j][0] = 0;
                grid_image[i][j][1] = 180;
                grid_image[i][j][2] = 180;
            }
            // Unexplored (0 or anything else)
            else
            {
                if(KINECT_SIM && maze[i][j]==1)
                {
                    grid_image[i][j][0] = 0;
                    grid_image[i][j][1] = 0;
                    grid_image[i][j][2] = 130;
                }
                else
                {
                    grid_image[i][j][0] = 0;
                    grid_image[i][j][1] = 0;
                    grid_image[i][j][2] = 50;
                }
            }

        }
    }

    // Draw the Origin (the center of the grid)
    j=GRIDSIZE/2-1;
    for(i=GRIDSIZE/2-3; i<GRIDSIZE/2+2; i++)
    {
        grid_image[i][j][0] = 0;
        grid_image[i][j][1] = 200;
        grid_image[i][j][2] = 200;
    }
    i=GRIDSIZE/2-1;
    for(j=GRIDSIZE/2-3; j<GRIDSIZE/2+2; j++)
    {
        grid_image[i][j][0] = 0;
        grid_image[i][j][1] = 200;
        grid_image[i][j][2] = 200;
    }

    // Display the goal
    if(goal_pos != NULL)
    {
        for(i=-2; i<=2; i++)
        {
            int x = goal_pos->x;
            int y = goal_pos->y;
            if(withinBounds(x + i, y + i))
            {
                grid_image[x + i][y + i][0] = 255;
                grid_image[x + i][y + i][1] = 0;
                grid_image[x + i][y + i][2] = 0;
            }
            if(withinBounds(x - i, y + i))
            {
                grid_image[x - i][y + i][0] = 255;
                grid_image[x - i][y + i][1] = 0;
                grid_image[x - i][y + i][2] = 0;
            }
        }
    }

    // Display where the robot has been later
    grid[curX][curY] = VISITED;

    // Display the robot's position in the map
    grid_image[curX][curY][0] = 255;
    grid_image[curX][curY][1] = 255;
    grid_image[curX][curY][2] = 255;

    // Draw the wingtips on the robot, provided they stay on the grid.
    int wingtips[8] = {0};
    getWingtips(curX, curY, wingtips, cur_pos);
    for(a=0; a<8; a+=2)
    {
        if(wingtips[a] >= 0 && wingtips[a+1] < GRIDSIZE
                && wingtips[a] >= 0 && wingtips[a+1] < GRIDSIZE)
        {
            grid_image[wingtips[a]][wingtips[a+1]][0] = 255;
            grid_image[wingtips[a]][wingtips[a+1]][1] = 255;
            grid_image[wingtips[a]][wingtips[a+1]][2] = 255;
        }
    }

}



void mouseMoved(int x, int y)
{
    if (mx>=0 && my>=0) {
        rotangles[0] += y-my;
        rotangles[1] += x-mx;
    }
    mx = x;
    my = y;
}

void mousePress(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        mx = x;
        my = y;
    }
    if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
        mx = -1;
        my = -1;
    }
}

void no_kinect_quit(void)
{
    printf("Error: Kinect not connected?\n");
    exit(1);
}



void keyPressed(unsigned char key, int x, int y)
{
    if (key == 27) {
        freenect_sync_stop();
        glutDestroyWindow(window);
        exit(0);
    }
    if (key == 'w')
        zoom *= 1.1f;
    if (key == 's')
        zoom /= 1.1f;
    if (key == 'c')
        color = !color;
}

void ReSizeGLScene(int Width, int Height)
{
    glViewport(0,0,Width,Height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho (0, 1000, 428, 0, -1.0f, 1.0f);
    //gluPerspective(60, 4/3., 0.3, 200);
    glMatrixMode(GL_MODELVIEW);
}

void InitGL(int Width, int Height)
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0);
    glDepthFunc(GL_LESS);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel(GL_SMOOTH);
    glGenTextures(1, &gl_depth_tex);
    glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenTextures(1, &gl_rgb_tex);
    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    ReSizeGLScene(Width, Height);
}

void set_curvature(float curvature, int speed)
{
    float rspeed_cm_per_s = (speed/100.0f) * TOP_SPEED_CM_PER_S *
        (ROBOT_RADIUS * curvature + 1);
    float rspeed = rspeed_cm_per_s * 100 / TOP_SPEED_CM_PER_S;
    float lspeed = 2 * speed - rspeed;

    //printf("Rspeed cm per s: %f, Rspeed: %f, Lspeed: %f\n", rspeed_cm_per_s, rspeed, lspeed);

    if(rspeed >= 100.5 || lspeed <= -100.5)
    {
        rspeed = 50;
        lspeed = -50;
    }
    else if(rspeed <= -100.5 || lspeed >= 100.5)
    {
        rspeed = -50;
        lspeed = 50;
    }
    set_motor_l((int)lspeed);
    set_motor_r((int)rspeed);
}

int goalReached()
{
    if(!withinBounds(goal_pos->x, goal_pos->y))
        return 0;

    if(grid[goal_pos->x][goal_pos->y] == EMPTY)
        return 0;

    return 1;
}

/**
 * Returns 1 if finished turning, 0 otherwise.
 */
int turnToPoint(gridPoint* point)
{
    float final_angle;

    final_angle = atan((cur_pos->y - point->y) / (cur_pos->x - point->x));

    if(abs(cur_pos->theta - final_angle) < PI/4)
    {
        set_curvature(.01, 0);
        return 1;
    }

    set_curvature(1.0, 50);
    return 0;
}

/**
 * Comes up with a plan to see the goal point, and executes that plan.
 */
void executePlan(int* depths)
{
    int curX = (int)(-(cur_pos->x/CELL_WIDTH) + (GRIDSIZE/2) - .5);
    int curY = (int)(-(cur_pos->y/CELL_WIDTH) + (GRIDSIZE/2) - .5);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Just drive at a curvature
    set_curvature(.003, 50);
    return;

        float depth = 0;
        float theta = cur_pos->theta;
        float m, m2, x, y, a;
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
                x = xSign * a + curX;
                y = m*(x - curX) + curY;
            }
            else
            {
                y = ySign * a + curY;
                x = m2*(y - curY) + curX;
            }

            gridX = (int)x;  //(-(x/CELL_WIDTH) + (GRIDSIZE/2) - .5);
            gridY = (int)y;  //(-(y/CELL_WIDTH) + (GRIDSIZE/2) - .5);

            if(gridX < 0 || gridX >= GRIDSIZE || gridY < 0 || gridY >= GRIDSIZE)
                break;

            if(grid[gridX][gridY] == BLOCKED_SEEN || grid[gridX][gridY] == BLOCKED_EMPTY)
            {
                depth = a;
                //           depth = sqrt((curX - gridX)*(curX - gridX) +
                //              (cur_pos->y - gridY)*(cur_pos->y - gridY));
                break;
            }
        }
        //printf("I see depth %f\n", depth);
        // Check for collision soon
        if(depth < 50)
            set_curvature(1.0, 50);
        else if(depth < 300)
            set_curvature(-.1, 50);
        else
            set_curvature(0, 50);

        /*    int goToX = curX;
              int goToY = curY;
              findNearest(-1, &goToX, &goToY);

              gridPoint goToPoint;
              goToPoint.x = goToX;
              goToPoint.y = goToY;

              if(turnToPoint(&goToPoint))
              {
              set_curvature(0, 50);
              }*/

        // Random movement?

    //set_curvature(0.03, 0);  // Stop

}

void time_step(int* depths)
{
    int i, j;
    float a, x, y, m, m2;
    float theta;

    float encoder_l;
    float encoder_r;

    // Clears the screen after some number of time_steps, if demo mode.
    if(DEMO_MODE)
    {
        count++;
        if(count > 150)
        {
            for(i=0; i<GRIDSIZE; i++)
            {
                for(j=0; j<GRIDSIZE; j++)
                {
                    grid[i][j] = 0;
                }
            }
            count = 0;
        }
    }

    // Drive to a new goal once we reach our current target
    if(goalReached() || goal_pos->x == -1)
    {
        printf("Goal reached!\n");
        setGoal(grid, cur_pos, goal_pos);
    }

    executePlan(depths);

    if(ENCODERS_SIM)
    {
        encoder_l = ((float)motor_l_speed) * TOP_SPEED_CM_PER_S *
            ENCODER_CLICKS_PER_CM / 100;
        encoder_r = ((float)motor_r_speed) * TOP_SPEED_CM_PER_S *
            ENCODER_CLICKS_PER_CM / 100;
    }
    else
    {
        encoders enc;
        read_encoders(&enc);
        encoder_l = (motor_l_speed < 0) ? -enc.left : enc.left;
        encoder_r = (motor_r_speed < 0) ? -enc.right : enc.right;
    }

    float dist_l = encoder_l / ENCODER_CLICKS_PER_CM;
    float dist_r = encoder_r / ENCODER_CLICKS_PER_CM;

    // Circular approximation
    float d_theta = (dist_r - dist_l)/(2 * ROBOT_RADIUS);
    //printf("EncL: %f, EncR: %f, DstL: %f, DstR: %f, D theta: %f\n", encoder_l, encoder_r, dist_l, dist_r, d_theta);

    float chord_len;
    if(d_theta < .0001) // Special case: nearly straight path
    {
        chord_len = dist_l;
    }
    else
    {
        float r_inner = dist_l/d_theta;
        chord_len = 2 * (r_inner + ROBOT_RADIUS) * sin(d_theta / 2);
    }

    float chord_theta = cur_pos->theta + (d_theta / 2);
    putAngleInBounds(&chord_theta);

    float dy = chord_len * sin(chord_theta);
    float dx = chord_len * cos(chord_theta);

    cur_pos->x = cur_pos->x + dx;
    cur_pos->y = cur_pos->y + dy;
    cur_pos->theta = cur_pos->theta + d_theta;
    putAngleInBounds(&(cur_pos->theta));

    if(KINECT_SIM)
        generateDepths(maze, cur_pos, depths);

    // For each depth, calculate the pose relative to our current position.
    for(i=0; i<640; i++)
    {
        if(depths[i] == -1)     // error code, no reading at this value
            continue;

        if(FULL_DISPLAY)
        {
            // Draw what we've seen up to the depth
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
                    x = xSign * a + cur_pos->x;
                    y = m*(x - cur_pos->x) + cur_pos->y;
                }
                else
                {
                    y = ySign * a + cur_pos->y;
                    x = m2*(y - cur_pos->y) + cur_pos->x;
                }

                if(sqrt(((x-cur_pos->x)*(x-cur_pos->x)) +
                        ((y-cur_pos->y)*(y-cur_pos->y))) > depths[i])
                    break;

                gridX = (-(x/CELL_WIDTH) + (GRIDSIZE/2) - .5);
                gridY = (-(y/CELL_WIDTH) + (GRIDSIZE/2) - .5);

                if(gridX < 0 || gridX >= GRIDSIZE || gridY < 0
                        || gridY >= GRIDSIZE)
                    break;

                if(grid[gridX][gridY] == EMPTY)
                    grid[gridX][gridY] = SEEN;
                else if(grid[gridX][gridY] == BLOCKED_EMPTY)
                    grid[gridX][gridY] = BLOCKED_SEEN;
            }
        }

        pose2D *point_seen = malloc(sizeof(pose2D));
        point_seen->theta = (((float)i)-320) / 320 * MAX_ANGLE;
        point_seen->x = depths[i] * cos(point_seen->theta);
        point_seen->y = depths[i] * sin(point_seen->theta);

        convertToFrame(point_seen, cur_pos);

        // The point is now in the standard world frame - draw it on the grid.
        drawReadingOnGrid(grid, point_seen);
    }
}

/**
 * Called in a loop constantly to draw the displays.
 */
void DrawGLScene()
{
    int depths[640] = {0};

    char *rgb = 0;
    uint32_t ts;
    int i,j;

    /* Displays the image on the left: the Kinect depth readings.
     * Only displays if the Kinect is plugged in (not in sim mode) */
    if (!KINECT_SIM)
    {
        if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
            no_kinect_quit();
        if (freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
            no_kinect_quit();

        for (i=0; i<FREENECT_FRAME_PIX; i++) {
            int pval = t_gamma[depth[i]];
            int lb = pval & 0xff;
            switch (pval>>8) {
                case 0:
                    depth_mid[3*i+0] = 255;
                    depth_mid[3*i+1] = 255-lb;
                    depth_mid[3*i+2] = 255-lb;
                    break;
                case 1:
                    depth_mid[3*i+0] = 255;
                    depth_mid[3*i+1] = lb;
                    depth_mid[3*i+2] = 0;
                    break;
                case 2:
                    depth_mid[3*i+0] = 255-lb;
                    depth_mid[3*i+1] = 255;
                    depth_mid[3*i+2] = 0;
                    break;
                case 3:
                    depth_mid[3*i+0] = 0;
                    depth_mid[3*i+1] = 255;
                    depth_mid[3*i+2] = lb;
                    break;
                case 4:
                    depth_mid[3*i+0] = 0;
                    depth_mid[3*i+1] = 255-lb;
                    depth_mid[3*i+2] = 255;
                    break;
                case 5:
                    depth_mid[3*i+0] = 0;
                    depth_mid[3*i+1] = 0;
                    depth_mid[3*i+2] = 255-lb;
                    break;
                default:
                    depth_mid[3*i+0] = 0;
                    depth_mid[3*i+1] = 0;
                    depth_mid[3*i+2] = 0;
                    break;
            }
        }
    }

    static unsigned int indices[480][640];
    static GLfloat xyz[480*640][4];


    // Sum up the bottom 80 realistic readings
    for (j = 0; j < 640; j++)
    {
        int numAccurateReadings = 0;

        for (i = 400; i < 480; i++)
        {
            
            int idx = RCW_TO_IDX(i,j,640);
            xyz[idx][0] = j;
            xyz[idx][1] = i;
            xyz[idx][2] = KINECT_SIM ? 0 : depth[idx];
            xyz[idx][3] = 1;
            indices[i][j] = idx;

            if(!KINECT_SIM)
            {
                if((int)depth[idx] > 5)
                {
                    depths[j] += (int)depth[idx];
                    numAccurateReadings++;
                }
            }
        }
        if(numAccurateReadings == 0)
            depths[j] = -1;
        else
            depths[j] = depths[j] / numAccurateReadings;
    }

    float fx = 594.21f;
    float fy = 591.04f;
    float a = -0.0030711f;
    float b = 3.3309495f;
    float cx = 339.5f;
    float cy = 242.7f;
    GLfloat mat[16] = {
        1/fx,     0,  0, 0,
        0,    -1/fy,  0, 0,
        0,       0,  0, a,
        -cx/fx, cy/fy, -1, b
    };

    /*
       GLfloat *convertedXYZ = multiplyMatrixTransposed(mat, (GLfloat*)xyz, 4, 4, 480*640);

       llAddLast((void*)convertedXYZ);
     */

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glEnable(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_mid);

    glBegin(GL_TRIANGLE_FAN);
    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    glTexCoord2f(0, 0); glVertex3f(0,0,0);
    glTexCoord2f(1, 0); glVertex3f(570,0,0);
    glTexCoord2f(1, 1); glVertex3f(570,428,0);
    glTexCoord2f(0, 1); glVertex3f(0,428,0);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, GRIDSIZE, GRIDSIZE, 0, GL_RGB, GL_UNSIGNED_BYTE, grid_image);

    glBegin(GL_TRIANGLE_FAN);
    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    glTexCoord2f(0, 0); glVertex3f(570,0,0);
    glTexCoord2f(1, 0); glVertex3f(570+428,0,0);
    glTexCoord2f(1, 1); glVertex3f(570+428,428,0);
    glTexCoord2f(0, 1); glVertex3f(570,428,0);
    glEnd();


    // Move the robot around, recalculate and such.
    staleGrid(grid);
    time_step(depths);
    displayGrid();
    //sleep(SIM_DELAY);
    //eraseGrid(grid);

    glutSwapBuffers();
}


int main(int argc, char **argv)
{
    init_comm(); // Start the serial connection
    loadMaze();  // For simulation: loads a maze for simulated depth readings

    cur_pos = malloc(sizeof(pose2D));
    cur_pos->x = 0;
    cur_pos->y = 0;
    cur_pos->theta = 0;

    goal_pos = malloc(sizeof(gridPoint));
    goal_pos->x = -1;
    goal_pos->y = -1;

    set_curvature(0.003, 50);
    //set_curvature(.003, 50);

    depth_mid = (uint8_t*)malloc(640*480*3);
    int i, j;
    for (i=0; i<2048; i++) {
        float v = i/2048.0;
        v = powf(v, 3)* 6;
        t_gamma[i] = v*6*256;
    }

    for (i=0; i<GRIDSIZE; i++) {
        for (j=0; j<GRIDSIZE; j++) {
            grid_image[i][j][0] = 0;
            grid_image[i][j][1] = 0;
            grid_image[i][j][2] = 0;
        }
    }

    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
    glutInitWindowSize(1000, 428);
    glutInitWindowPosition(0, 0);

    window = glutCreateWindow("Mapkin");

    glutDisplayFunc(&DrawGLScene);
    glutIdleFunc(&DrawGLScene);
    glutReshapeFunc(&ReSizeGLScene);
    glutKeyboardFunc(&keyPressed);
    glutMotionFunc(&mouseMoved);
    glutMouseFunc(&mousePress);

    InitGL(1000, 428);

    glutMainLoop();

    return 0;
}
