/**
 * A utilities file for matrix operations. This includes openGL-written
 * functions to change the openGL stored matrix, as well as my own functions
 * to multiply matrices as needed for coordinate frame conversion.
 *
 * @author Alex Zirbel
 */

#include "mapkin.h"

//////////////////////////////////////////////////////////////////////////////////
// OpenGL Functions
//////////////////////////////////////////////////////////////////////////////////

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




//////////////////////////////////////////////////////////////////////////////////
// MY FUNCTIONS FOR FRAME CONVERSION AND MANIPULATING MATRICES
//////////////////////////////////////////////////////////////////////////////////

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

/**
 * Prints out the data in a matrix of given height and width, for debugging.
 */
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
