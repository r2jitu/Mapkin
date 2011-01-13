/*
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

#include "libfreenect.h"
#include "libfreenect_sync.h"
#include <stdio.h>
#include <stdlib.h>

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
#define GRIDSIZE 25

int window;
GLuint gl_rgb_tex;
int mx=-1,my=-1;        // Prevous mouse coordinates
int rotangles[2] = {0}; // Panning angles
float zoom = 1;         // zoom factor
int color = 1;          // Use the RGB texture or just draw it as color
int count = 0;

int grid[GRIDSIZE][GRIDSIZE] = {{0}};   // Mark the places we have explored and seen a wall

typedef struct llnode {
    struct llnode *next, *prev;
    void *data;
} llnode;

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


void forward(float dist)
{
    cur_pos->x = cur_pos->x + dist;
}
void left(float dist)
{
    cur_pos->y = cur_pos->y + dist;
}

void displayGrid()
{
    int i=0, j=0;
    int curX = (int)(-cur_pos->x + (GRIDSIZE/2) - .5);
    int curY = (int)(-cur_pos->y + (GRIDSIZE/2) - .5);
    
    printf("Current position: [%f %f %f]\n", cur_pos->x, cur_pos->y, cur_pos->theta);

    for(i=0; i<GRIDSIZE; i++)
    {
        for(j=0; j<GRIDSIZE; j++)
        {
            if(i==curX && j==curY)
                printf("X ");
            else
                printf(". ");

        }
        printf("\n");
    }
    printf("\n");
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

void DrawGLScene()
{
    short *depth = 0;
    char *rgb = 0;
    uint32_t ts;
    if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
        no_kinect_quit();
    if (freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
        no_kinect_quit();

    static unsigned int indices[480][640];
    static GLfloat xyz[480*640][4];
    int i,j;
    for (i = 0; i < 480; i++) {
        for (j = 0; j < 640; j++) {
            int idx = RCW_TO_IDX(i,j,640);
            xyz[idx][0] = j;
            xyz[idx][1] = i;
            xyz[idx][2] = depth[idx];
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

    GLfloat *convertedXYZ = multiplyMatrixTransposed(mat, (GLfloat*)xyz, 4, 4, 480*640);

    llAddLast((void*)convertedXYZ);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glPushMatrix();
    glScalef(zoom,zoom,1);
    glTranslatef(0,0,-3.5);
    glRotatef(rotangles[0], 1,0,0);
    glRotatef(rotangles[1], 0,1,0);
    glTranslatef(0,0,1.5);

    LoadVertexMatrix();

    // Set the projection from the XYZ to the texture image
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glScalef(1/640.0f,1/480.0f,1);
    LoadRGBMatrix();
    LoadVertexMatrix();
    glMatrixMode(GL_MODELVIEW);

    glPointSize(1);

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(4, GL_FLOAT, 0, xyz);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(4, GL_FLOAT, 0, xyz);

    if (color)
        glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb);

    glPointSize(2.0f);
    glDrawElements(GL_POINTS, 640*480, GL_UNSIGNED_INT, indices);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);

    printf("..");
    sleep(1);
    count++;
    printf("Num captures: %d\n", count);

    glutSwapBuffers();
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
    gluPerspective(60, 4/3., 0.3, 200);
    glMatrixMode(GL_MODELVIEW);
}

void InitGL(int Width, int Height)
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glEnable(GL_DEPTH_TEST);
    glGenTextures(1, &gl_rgb_tex);
    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    ReSizeGLScene(Width, Height);
}

/*int main(int argc, char **argv)
{
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutInitWindowPosition(0, 0);

    window = glutCreateWindow("LibFreenect");

    glutDisplayFunc(&DrawGLScene);
    glutIdleFunc(&DrawGLScene);
    glutReshapeFunc(&ReSizeGLScene);
    glutKeyboardFunc(&keyPressed);
    glutMotionFunc(&mouseMoved);
    glutMouseFunc(&mousePress);

    InitGL(640, 480);

    glutMainLoop();

    return 0;
}*/

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

// To test filling in the grid.
int main(int argc, char** argv)
{
    cur_pos = malloc(sizeof(pose2D));
    cur_pos->x = 0;
    cur_pos->y = 0;
    cur_pos->theta = 0;
//    cur_pos->data = {GRIDSIZE/2, GRIDSIZE/2, 0};

    while(1)
    {
        forward(.2);
        displayGrid();
        sleep(.3);
    }

    return 0;
}
