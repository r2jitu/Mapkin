
#include "grid_main.h"

int window;
GLuint gl_depth_tex;
GLuint gl_rgb_tex;
int mx=-1,my=-1;        // Prevous mouse coordinates
int rotangles[2] = {0}; // Panning angles
float zoom = 1;         // zoom factor
int color = 1;          // Use the RGB texture or just draw it as color
int count = 0;
int die = 0;

int motor_l_speed = 0;
int motor_r_speed = 0;

int grid[GRIDSIZE][GRIDSIZE] = {{0}};   // Mark the places we have explored and seen a wall
char grid_image[GRIDSIZE][GRIDSIZE][3];

typedef struct llnode {
    struct llnode *next, *prev;
    void *data;
} llnode;

uint8_t *depth_mid;
uint16_t t_gamma[2048];
llnode *head = NULL, *tail = NULL;

typedef struct pose2D {
    float x, y, theta;
    //float x, y, theta;
    //GLfloat data[3]; // x, y, theta
} pose2D;

pose2D *cur_pos = NULL;


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
 * Fills in an array of integer depths (assumed to be 640 integers, for a
 * Kinect) with some generated sequence of numbers.
 */
void generateDepths(int* depths)
{
    int i;
    for(i=0; i<640; i++)
    {
        depths[i] = 240;
    }
}

/**
 * Multiplies two matrices with the specified dimensions.
 * m1w is the width of mat1, etc.
 */
GLfloat* multiplyMatrix(GLfloat* mat1, GLfloat* mat2,
        int m1w, int m1h, int m2w)
{
    int i=0, j=0, a=0;
    GLfloat* result = malloc((sizeof(float) * m1h * m2w));

    for(i=0; i<m1h; i++)
    {
        for(j=0; j<m2w; j++)
        {
            float newVal = 0;
            for(a = 0; a < m1w; a++)
            {
                newVal += (mat1[RCW_TO_IDX(i,a,m1w)] * mat2[RCW_TO_IDX(a,j,m2w)]);
            }
            result[RCW_TO_IDX(i,j,m2w)] = newVal;
        }
    }

    return result;
}

// Do the projection from u,v,depth to X,Y,Z directly in an opengl matrix
// These numbers come from a combination of the ros kinect_node wiki, and
// nicolas burrus' posts.
void LoadVertexMatrix()
{
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
    glMultMatrixf(mat);
}

/**
 * A quick way to transpose the second matrix, then multiply matrices as above.
 */
GLfloat* multiplyMatrixTransposed(GLfloat* mat1, GLfloat* mat2,
        int m1w, int m1h, int m2h)
{
    int i=0, j=0, a=0;
    GLfloat* result = malloc((sizeof(float) * m1h * m2h));

    for(i=0; i<m1h; i++)
    {
        for(j=0; j<m2h; j++)
        {
            float newVal = 0;
            for(a = 0; a < m1w; a++)
            {
                newVal += (mat1[RCW_TO_IDX(i,a,m1w)] * mat2[RCW_TO_IDX(j,a,m1w)]);
            }
            result[RCW_TO_IDX(i,j,m2h)] = newVal;
        }
    }

    return result;
}

void displayMatrix(GLfloat* mat, int h, int w)
{
    int i=0;

    for(i=0; i<h*w; i++)
    {
        if(i % w == 0 && i != 0)
            printf("\n");
        printf("%f ", mat[i]);
    }
    printf("\n");
}

void displayPose2D(pose2D *pose)
{
    printf("[%f, %f, %f]", pose->x, pose->y, pose->theta);
}

/**
 * Assuming the point is in the right frame (relative to the grid, with x upward
 * in the center of the grid, y left, theta counterclockwise from x
 *
 * Code: 0 is empty. 1 is wall. 2 is current sensor reading.
 * 3 is undrivable area. 4 is seen area. 5 is where the robot has already gone.
 */
void drawOnGrid(pose2D *point)
{

    int x = (-(point->x/CELL_WIDTH) + (GRIDSIZE/2) - .5);
    int y = (-(point->y/CELL_WIDTH) + (GRIDSIZE/2) - .5);
   // (GRIDSIZE/2) - (int)((point->x + .5)/CELL_WIDTH) - 1;
    //int y = (GRIDSIZE/2) - (int)((point->y + .5)/CELL_WIDTH) - 1;

    if(x >= 0 && x < GRIDSIZE && y >= 0 && y < GRIDSIZE)
        grid[x][y] = 2;
}

/**
 * The current sensor reading spots on the grid (2) become old readings (1).
 */
void staleGrid()
{
    int i, j;
    for(i=0; i<GRIDSIZE; i++)
    {
        for(j=0; j<GRIDSIZE; j++)
        {
            if(grid[i][j] == 2)
                grid[i][j] = 1;
        }
    }
}

void eraseGrid()
{
    int i, j;
    for(i=0; i<GRIDSIZE; i++)
    {
        for(j=0; j<GRIDSIZE; j++)
        {
            grid[i][j] = 0;
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

void getWingtips(int i, int j, int* wingtips)
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

void displayGrid()
{
    int i=0, j=0, a=0;
    int curX = (int)(-(cur_pos->x/CELL_WIDTH) + (GRIDSIZE/2) - .5);
    int curY = (int)(-(cur_pos->y/CELL_WIDTH) + (GRIDSIZE/2) - .5);

    printf("Current position: [%f %f %f]\n", cur_pos->x, cur_pos->y, cur_pos->theta);
    printf("Current speed: [%d %d]\n", motor_l_speed, motor_r_speed);

    for(i=0; i<GRIDSIZE; i++)
    {
        for(j=0; j<GRIDSIZE; j++)
        {
            // Old wall seen here
            if(grid[i][j] == 1)
            {
                grid_image[i][j][0] = 0;
                grid_image[i][j][1] = 0;
                grid_image[i][j][2] = 0;
            }
            // Current sensor reading
            else if(grid[i][j] == 2)
            {
                grid_image[i][j][0] = 255;
                grid_image[i][j][1] = 0;
                grid_image[i][j][2] = 0;
            }
            // Unexplored
            else
            {
                grid_image[i][j][0] = 0;
                grid_image[i][j][1] = 0;
                grid_image[i][j][2] = 50;
            }
        }
    }

    // The Origin
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

    // Display the robot's position in the map
    grid_image[curX][curY][0] = 255;
    grid_image[curX][curY][1] = 255;
    grid_image[curX][curY][2] = 255;

    int wingtips[8] = {0};
    getWingtips(curX, curY, wingtips);
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


// This matrix comes from a combination of nicolas burrus's calibration post
// and some python code I haven't documented yet.
void LoadRGBMatrix()
{
    float mat[16] = {
        5.34866271e+02,   3.89654806e+00,   0.00000000e+00,   1.74704200e-02,
        -4.70724694e+00,  -5.28843603e+02,   0.00000000e+00,  -1.22753400e-02,
        -3.19670762e+02,  -2.60999685e+02,   0.00000000e+00,  -9.99772000e-01,
        -6.98445586e+00,   3.31139785e+00,   0.00000000e+00,   1.09167360e-02
    };
    glMultMatrixf(mat);
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



/*
   int main(int argc, char **argv)
   {

   GLfloat mat1[16] = {
   1.455,     0,  0, 0,
   0,    -1.441,  0, 0,
   0,       0,  0, 4.55,
   -2.33, 2.33, -1, 4
   };

   GLfloat mat2[16] = {
   0, 0, 0, 1,
   0, 0, 1, 0,
   0, 1, 0, 0,
   1, 0, 0, 0 
   };

   GLfloat ID[16] = {
   1, 0, 0, 0,
   0, 1, 0, 0,
   0, 0, 1, 0,
   0, 0, 0, 1 
   };

   GLfloat mat3[6] = {
   1, 2, 3,
   2, 3, 4
   };

   GLfloat mat4[6] = {
   3, 3,
   2, 3,
   8, 3
   };

   GLfloat* result = multiplyMatrix(mat3, mat4, 3, 2, 2);

   printf("\n");
   displayMatrix(result, 2, 2);
   printf("\n");

   return 0;
   }
 */

void set_motor_l(int speed)
{
    if(speed < -100 || speed > 100)
    {
        printf("Speed out of bounds.\n");
        die = 1;
        return;
    }

    motor_l_speed = speed;
}

void set_motor_r(int speed)
{
    if(speed < -100 || speed > 100)
    {
        printf("Speed out of bounds.\n");
        die = 1;
        return;
    }

    motor_r_speed = speed;
}

void set_curvature(float curvature, int speed)
{
    float rspeed_cm_per_s = (speed/100.0f) * TOP_SPEED_CM_PER_S *
        (ROBOT_RADIUS * curvature + 1);
    float rspeed = rspeed_cm_per_s * 100 / TOP_SPEED_CM_PER_S;
    float lspeed = 2 * speed - rspeed;

    printf("Rspeed cm per s: %f, Rspeed: %f, Lspeed: %f\n", rspeed_cm_per_s, rspeed, lspeed);

    if(rspeed >= 100.5 || lspeed <= -100.5)
    {
        rspeed = 100;
        lspeed = -100;
    }
    else if(rspeed <= -100.5 || lspeed >= 100.5)
    {
        rspeed = -100;
        lspeed = 100;
    }
    set_motor_l((int)lspeed);
    set_motor_r((int)rspeed);
}

void time_step()
{
    int i;
    float encoder_l = ((float)motor_l_speed) * TOP_SPEED_CM_PER_S * ENCODER_CLICKS_PER_CM / 100;
    float encoder_r = ((float)motor_r_speed) * TOP_SPEED_CM_PER_S * ENCODER_CLICKS_PER_CM / 100;
    float dist_l = encoder_l / ENCODER_CLICKS_PER_CM;
    float dist_r = encoder_r / ENCODER_CLICKS_PER_CM;

    // Linear approximation
    /*    float d_theta = atan((dist_r - dist_l) / (2 * ROBOT_RADIUS));
          printf("EncL: %f, EncR: %f, DstL: %f, DstR: %f, D theta: %f\n", encoder_l, encoder_r, dist_l, dist_r, d_theta);
          float dy = 0;
          float dx = (dist_r + dist_l) / 2;*/

    // Circular approximation
    float d_theta = (dist_r - dist_l)/(2 * ROBOT_RADIUS);
    printf("EncL: %f, EncR: %f, DstL: %f, DstR: %f, D theta: %f\n", encoder_l, encoder_r, dist_l, dist_r, d_theta);
    float r_inner = dist_l/d_theta;
    float chord_theta = cur_pos->theta + (d_theta / 2);
    putAngleInBounds(&chord_theta);

    float chord_len = 2 * (r_inner + ROBOT_RADIUS) * sin(d_theta / 2);

    //float dy = chord_len * sin(chord_theta / 2);
    //float dx = chord_len * cos(chord_theta / 2);
    float dy = chord_len * sin(chord_theta);
    float dx = chord_len * cos(chord_theta);

    cur_pos->x = cur_pos->x + dx;
    cur_pos->y = cur_pos->y + dy;
    cur_pos->theta = cur_pos->theta + d_theta;
    putAngleInBounds(&(cur_pos->theta));


    // Read the kinect to get depths and plot this on the map
    int depths[640] = {0};
    generateDepths(depths);

    // For each depth, calculate the pose relative to our current position.
    for(i=0; i<640; i++)
    {
        float max_angle = PI / 8;
        pose2D *point_seen = malloc(sizeof(pose2D));
        point_seen->theta = (((float)i)-320) / 320 * max_angle;
        point_seen->x = depths[i] * cos(point_seen->theta);
        point_seen->y = depths[i] * sin(point_seen->theta);

        convertToFrame(point_seen, cur_pos);

        // The point is now in the standard world frame - draw it on the grid.
        drawOnGrid(point_seen);
    }

}







// To test filling in the grid.
/*
   int main(int argc, char** argv)
   {
   cur_pos = malloc(sizeof(pose2D));
   cur_pos->x = 0;
   cur_pos->y = 0;
   cur_pos->theta = 0;
//    cur_pos->data = {GRIDSIZE/2, GRIDSIZE/2, 0};

set_curvature(.01, 50);

while(!die)
{
time_step();
displayGrid();
sleep(1);
}

return 0;
}
 */

void DrawGLScene()
{
    // Move the robot around, recalculate and such.
    staleGrid();
    time_step();
    displayGrid();
    sleep(SIM_DELAY);
    //eraseGrid();

    short *depth = 0;
    char *rgb = 0;
    uint32_t ts;
    int i,j;

    if (KINECT_ATTACHED)
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
    for (i = 0; i < 480; i++) {
        for (j = 0; j < 640; j++) {
            int idx = RCW_TO_IDX(i,j,640);
            xyz[idx][0] = j;
            xyz[idx][1] = i;
            xyz[idx][2] = KINECT_ATTACHED ? depth[idx] : 0;
            xyz[idx][3] = 1;
            indices[i][j] = idx;
        }
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

    glutSwapBuffers();
}


int main(int argc, char **argv)
{
    cur_pos = malloc(sizeof(pose2D));
    cur_pos->x = 0;
    cur_pos->y = 0;
    cur_pos->theta = 0;
    set_curvature(.003, 50);

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

    window = glutCreateWindow("LibFreenect");

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

// Test converting between coordinate frames.
/*int main(int argc, char** argv)
{
    pose2D *robot_frame = malloc(sizeof(pose2D));
    robot_frame->x = 3;
    robot_frame->y = 5;
    robot_frame->theta = PI/2;
    pose2D *point_in_robot = malloc(sizeof(pose2D));
    point_in_robot->x = 10;
    point_in_robot->y = 10;
    point_in_robot->theta = PI;

    printf("Original point, in robot frame ");
    displayPose2D(robot_frame);
    printf(":\n");
    displayPose2D(point_in_robot);
    printf("\nPoint in world frame:\n");
    convertToFrame(point_in_robot, robot_frame);
    displayPose2D(point_in_robot);
    printf("\n");
    return 0;
}*/