//
//  main.c
//  A1
//
//  Created by Johna Latouf on 2018-09-21.
//  Switch between interpolations with keys:
//  Slerp (s), Catmull Rom (c), Bessel-Overhauser (b)
//  Runs at 25 fps, change FRAME_RATE to increase or decrease

// to compile this on mac you have to use this:
// gcc -std=c99 -o a1output main.c -framework OpenGL -framework GLUT
//

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <stdio.h>

/////////////////////////////////// type definitions ////////////////////////////////////

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180.0/M_PI
#define DEG2RAD M_PI/180.0


#define CAM_DIST 10.0
#define MAX_POINTS    512
#define MAX_VERTS 1000
#define SMOOTH 16
#define MAX_BOXES 5
#define MAX_SECONDS 20
#define FRAME_RATE 25


/////////////////////////////////// structs //////////////////////////////////////////

struct Vector {
    double x;
    double y;
    double z;
};

struct Quaternion {
    double s;
    struct Vector v;
};


/////////////////////////////////// function prototypes //////////////////////////////////

static void    myDisplay(void);
static void    myTimer(int value);
static void    myKey(unsigned char key, int x, int y);
static void    myReshape(int w, int h);

static void    init(void);

static void drawFloor(void);
static void drawSky(void);
static void drawBoxes(void);
void keyframeLoop(void);
void slerpLoop(void);
void curveLoopBO(void);
void curveLoopCR(void);

double v_dot(struct Vector v1, struct Vector v2);
struct Vector v_cross(struct Vector v1, struct Vector v2);
struct Vector v_sum(struct Vector v1, struct Vector v2);
struct Vector v_scalar(double s, struct Vector v);
struct Vector v_normalize(struct Vector v);
double q_dot(struct Quaternion q1, struct Quaternion q2);
struct Quaternion q_sum(struct Quaternion q1, struct Quaternion q2);
struct Quaternion q_mult(struct Quaternion q1, struct Quaternion q2);
struct Quaternion q_scalar(double s, struct Quaternion q);
struct Quaternion q_invert(struct Quaternion q);
struct Vector q_v_rotate(struct Quaternion q, struct Vector v);
struct Quaternion q_slerp(struct Quaternion q1, struct Quaternion q2, double t);
struct Quaternion q_pow(struct Quaternion q, double exp);
struct Quaternion q_exp(struct Quaternion q);
struct Quaternion q_log(struct Quaternion q);
double q_distance(struct Quaternion q1, struct Quaternion q2);
double q_rot_distance(struct Quaternion q1, struct Quaternion q2);
void q_to_axis(struct Quaternion q, float * angle, float * x, float * y, float * z);
struct Quaternion q_normalize(struct Quaternion q);
double q_magnitude(struct Quaternion q);
double q_magnitude_between(struct Quaternion q1, struct Quaternion q2);
struct Quaternion q_closer(struct Quaternion q1, struct Quaternion q2);
double choose(double n, double k);

void printSlerp(void);
void printCurveBO(void);
void printCurveCR(void);
void printKeyframes(void);

// for debugging
void printCTRL(void);

/////////////////////////////////// global variables ////////////////////////////////////

static double    xMax, yMax;

double eyex, eyey, eyez, upx, upy, upz;                     // camera eye
double distance;                                            // camera distance from center
double camAngleX, camAngleY;
GLfloat lookx, looky, lookz;

float movebox = 0.0;

// for testing
GLfloat curveVerts[MAX_VERTS][3];                           // holds the vertex values
GLfloat q_points[MAX_POINTS][MAX_BOXES*4];                  // holds the quaternion values
GLfloat q_points_by_box[MAX_BOXES][MAX_POINTS][4];          // holds quaternions by box
int numVerts;                                               // total verts in curve verts
GLfloat rotator = 0.0;
int curve_type = 0;                                         // use keys to switch between curve types
struct Quaternion rotation_q;
float axis_angle[MAX_BOXES];
float axis_x[MAX_BOXES];
float axis_y[MAX_BOXES];
float axis_z[MAX_BOXES];

// current time
int cur_t = 0;

// for reading file
int numPoints;
int numBoxes;
GLfloat boxLength;
GLfloat q_times[MAX_POINTS];
int numLines;
int numFrames;

// keep track of box vectors for animating
float boxVectors[MAX_BOXES][72];

// interpolated positions
GLfloat interpSlerp[MAX_BOXES][MAX_POINTS*FRAME_RATE*MAX_SECONDS][4];
GLfloat interpCurveBO[MAX_BOXES][MAX_POINTS*FRAME_RATE*MAX_SECONDS][4];
GLfloat interpCurveCR[MAX_BOXES][MAX_POINTS*FRAME_RATE*MAX_SECONDS][4];

// for debugging
GLfloat ctrlPoints[MAX_BOXES][MAX_POINTS*FRAME_RATE*MAX_SECONDS][4];
/////////////////////////////////// callback functions ///////////////////////////////////

void myDisplay()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
    
    // Render a color-cube consisting of 6 quads with different colors
    glLoadIdentity();
    /* camera setup */
    gluLookAt(eyex, eyey, eyez,
              lookx, looky, lookz,
              upx, upy, upz);
    
    ///drawSky();
    
    //drawFloor();
    
    drawBoxes();
    
    glutSwapBuffers();
}

void myTimer(int value)
{
    
    //keyframeLoop();
    if (curve_type == 0) {
        slerpLoop();
        //printf("SLERP\n");
    } else if (curve_type == 1) {
        curveLoopCR();
        //printf("CR\n");
    } else {
        curveLoopBO();
        //printf("BO\n");
    }
    
    glutPostRedisplay();
    
    glutTimerFunc(1000/FRAME_RATE, myTimer, value);
}

// interpolate the keyframes with Bessel-Overhauser splines and put in interpCurve
void curveKeyframesBO() {
    // for every box b
    for (int b=0; b<numBoxes; b++) {
        int frame_idx = 0; // keeps track of each frame
        // for every point p
        for (int p=0; p<numPoints; p++) {
            // convert to quaternion structs
            struct Quaternion q1, q2, q0, q3;
            struct Quaternion q; // the output q
            
            double dist12, dist01, dist02, dist13, dist23, dist03;
            
            // Rather than drop the first and last points, loop
            // around so that the plots line up better
            if (p == 0) {
                
                double t1 = (q_times[p] - q_times[0]);
                double t2 = (q_times[p+1] - q_times[0]);
                double time = round((t2 - t1) * (float)FRAME_RATE);
                //now slerp for each for each time t
                for (int f=0; f<time; f++) {
                    
                    interpCurveBO[b][frame_idx][0] = q_points_by_box[b][0][0];
                    interpCurveBO[b][frame_idx][1] = q_points_by_box[b][0][1];
                    interpCurveBO[b][frame_idx][2] = q_points_by_box[b][0][2];
                    interpCurveBO[b][frame_idx][3] = q_points_by_box[b][0][3];
                    frame_idx++;
                }
                continue;
            } else if (p >= numPoints-2) {
                
                double t1 = (q_times[p] - q_times[0]);
                double t2 = (q_times[p+1] - q_times[0]);
                double time = round((t2 - t1) * (float)FRAME_RATE);
                //now slerp for each for each time t
                for (int f=0; f<time; f++) {
                    
                    interpCurveBO[b][frame_idx][0] = q_points_by_box[b][p][0];
                    interpCurveBO[b][frame_idx][1] = q_points_by_box[b][p][1];
                    interpCurveBO[b][frame_idx][2] = q_points_by_box[b][p][2];
                    interpCurveBO[b][frame_idx][3] = q_points_by_box[b][p][3];
                    frame_idx++;
                }
                continue;
            } else {
                q3.s = q_points_by_box[b][p+2][0];
                q3.v.x = q_points_by_box[b][p+2][1];
                q3.v.y = q_points_by_box[b][p+2][2];
                q3.v.z = q_points_by_box[b][p+2][3];
                
                q1.s = q_points_by_box[b][p][0];
                q1.v.x = q_points_by_box[b][p][1];
                q1.v.y = q_points_by_box[b][p][2];
                q1.v.z = q_points_by_box[b][p][3];
                
                q2.s = q_points_by_box[b][p+1][0];
                q2.v.x = q_points_by_box[b][p+1][1];
                q2.v.y = q_points_by_box[b][p+1][2];
                q2.v.z = q_points_by_box[b][p+1][3];
                
                q0.s = q_points_by_box[b][p-1][0];
                q0.v.x = q_points_by_box[b][p-1][1];
                q0.v.y = q_points_by_box[b][p-1][2];
                q0.v.z = q_points_by_box[b][p-1][3];
                
                dist12 = (q_times[p+1] - q_times[p]) * (float)FRAME_RATE;
                dist01 = (q_times[p] - q_times[p-1]) * (float)FRAME_RATE;
                dist02 = (q_times[p+1] - q_times[p-1]) * (float)FRAME_RATE;
                dist13 = (q_times[p+2] - q_times[p]) * (float)FRAME_RATE;
                dist23 = (q_times[p+2] - q_times[p+1]) * (float)FRAME_RATE;
                dist03 = (q_times[p+2] - q_times[p-1]) * (float)FRAME_RATE;
            }
            
            // convert the times
            double t1 = (q_times[p] - q_times[0]);
            double t2 = (q_times[p+1] - q_times[0]);
            double time = round((t2 - t1) * (float)FRAME_RATE);
            
            // ti+1/2 and t-1/2 appear to be quaternions
            // mult becomes exponent, sum becomes mult,
            // difference becomes mult by inverse
            // use these to get ti
            struct Quaternion t1plushalf, t1minushalf, ti1, t2plushalf, t2minushalf, ti2;
            
            if (dist12 == 0) {
                // dist is zero, duplicate
                q = q1;
                break;
            } else {
                t1plushalf = q_pow(q_mult(q_invert(q1), q2), 1.0/dist12);
                t2minushalf = q_pow(q_mult(q_invert(q1), q2), 1.0/dist12);
            }
            
            if (dist13 == 0) {
                // dist is zero, duplicate
                q = q1;
                break;
            } else {
                t2plushalf = q_pow(q_mult(q_invert(q2), q3), 1.0/dist23);
            }
            
            if (dist01 == 0) {
                // dist is zero, duplicate
                q = q1;
                break;
            } else {
                t1minushalf = q_pow(q_mult(q_invert(q0), q1), 1.0/dist01);
            }
            
            struct Quaternion ti_top = q_mult( q_pow(t1minushalf, dist12), q_pow(t1plushalf, dist01));
            struct Quaternion ti2_top = q_mult(q_pow(t2minushalf, dist23), q_pow(t2plushalf, dist12));
            
            if (dist02 == 0) {
                // dist is zero, duplicate
                q = q1;
                break;
            } else {
                ti1 = q_pow(ti_top, 1.0/dist02);
            }
            
            if (dist13 == 0) {
                // dist is zero, duplicate
                q = q1;
                break;
            } else {
                ti2 = q_pow(ti2_top, 1.0/dist13);
            }
            
            // then get the + and - for both
            struct Quaternion q1plus, q2minus;
            q1plus = q_mult(q1, q_pow(ti1, dist12/3.0));
            q2minus = q_mult(q2, q_pow(ti2, -dist12/3.0));
            
            struct Quaternion all_qs[4] = {q1, q1plus, q2minus, q2};
            
            //now slerp for each for each time t
            for (int f=0; f<time; f++) {
                double t = f/time;
                
                struct Quaternion w, prod;
                
                // the bk,n values
                double bnk_calc[4];
                bnk_calc[0] = (1 - t) * (1 - t) * (1 - t);
                bnk_calc[1] = 3 * t * (1 - t) * (1 - t);
                bnk_calc[2] = 3 * t * t * (1 - t);
                bnk_calc[3] = t * t * t;
                
                // and I guess here the rest is a bezier segment between
                // q1, q1plus, q2minus, q2... so do that next?
                // slide 92
                for (int qidx=1; qidx<4; qidx++) {
                    w = q_log(q_mult(q_invert(all_qs[qidx-1]), all_qs[qidx]));
                    double bnk = 0;
                    
                    for (int bnk_i=qidx; bnk_i<4; bnk_i++) {
                        bnk += bnk_calc[bnk_i];
                    }
                    
                    
                    // product from slide 82
                    if (qidx == 1) {
                        prod = q_exp(q_scalar(bnk, w)); // probably not working?
                    } else {
                        //prod = q_mult(q_pow(w, bnk), prod);
                        prod = q_mult(prod, q_exp(q_scalar(bnk, w)));
                    }
                    
                }
                
                // TODO - q = q1 * prod
                q = q_mult(q1, prod);
                
                // add the new q to the array
                interpCurveBO[b][frame_idx][0] = q.s;
                interpCurveBO[b][frame_idx][1] = q.v.x;
                interpCurveBO[b][frame_idx][2] = q.v.y;
                interpCurveBO[b][frame_idx][3] = q.v.z;
                
                // debugging
                ctrlPoints[b][frame_idx][0] = q1plus.s;
                ctrlPoints[b][frame_idx][1] = q1plus.v.x;
                ctrlPoints[b][frame_idx][2] = q1plus.v.y;
                ctrlPoints[b][frame_idx][3] = q1plus.v.z;
                
                frame_idx++;
            }
            
            
            // error checking
            if (frame_idx > MAX_POINTS*FRAME_RATE*MAX_SECONDS) {
                printf("too many frames, cutting this short\n");
                break;
            }
        }
        numFrames = (frame_idx - 1);
        printf("numFrames = %d", numFrames);
    }
}

// interpolate the keyframes with Bessel-Overhauser splines and put in interpCurve
void curveKeyframesCR() {
    // for every box b
    for (int b=0; b<numBoxes; b++) {
        int frame_idx = 0; // keeps track of each frame
        // for every point p
        for (int p=0; p<numPoints; p++) {
            // convert to quaternion structs
            struct Quaternion q1, q2, q0, q3;
            struct Quaternion q; // the output q
            
            if (p == 0) {
                
                double t1 = (q_times[p] - q_times[0]);
                double t2 = (q_times[p+1] - q_times[0]);
                double time = round((t2 - t1) * (float)FRAME_RATE);
                //now slerp for each for each time t
                for (int f=0; f<time; f++) {
                
                    interpCurveCR[b][frame_idx][0] = q_points_by_box[b][0][0];
                    interpCurveCR[b][frame_idx][1] = q_points_by_box[b][0][1];
                    interpCurveCR[b][frame_idx][2] = q_points_by_box[b][0][2];
                    interpCurveCR[b][frame_idx][3] = q_points_by_box[b][0][3];
                    frame_idx++;
                }
                continue;
                
            } else if (p == numPoints-2) {
                
                double t1 = (q_times[p] - q_times[0]);
                double t2 = (q_times[p+1] - q_times[0]);
                double time = round((t2 - t1) * (float)FRAME_RATE);
                //now slerp for each for each time t
                for (int f=0; f<time; f++) {
                    
                    interpCurveCR[b][frame_idx][0] = q_points_by_box[b][p][0];
                    interpCurveCR[b][frame_idx][1] = q_points_by_box[b][p][1];
                    interpCurveCR[b][frame_idx][2] = q_points_by_box[b][p][2];
                    interpCurveCR[b][frame_idx][3] = q_points_by_box[b][p][3];
                    frame_idx++;
                }
                continue;
                

            } else {
                q3.s = q_points_by_box[b][p+2][0];
                q3.v.x = q_points_by_box[b][p+2][1];
                q3.v.y = q_points_by_box[b][p+2][2];
                q3.v.z = q_points_by_box[b][p+2][3];
                
                q1.s = q_points_by_box[b][p][0];
                q1.v.x = q_points_by_box[b][p][1];
                q1.v.y = q_points_by_box[b][p][2];
                q1.v.z = q_points_by_box[b][p][3];
                
                q2.s = q_points_by_box[b][p+1][0];
                q2.v.x = q_points_by_box[b][p+1][1];
                q2.v.y = q_points_by_box[b][p+1][2];
                q2.v.z = q_points_by_box[b][p+1][3];
                
                // qi-1 is q0 in this case
                q0.s = q_points_by_box[b][p-1][0];
                q0.v.x = q_points_by_box[b][p-1][1];
                q0.v.y = q_points_by_box[b][p-1][2];
                q0.v.z = q_points_by_box[b][p-1][3];
            }
            
            // then get the + and - for both
            struct Quaternion q1plus, q2minus;
            
            // control points
            q1plus = q_mult(q1, q_pow(q_mult(q_invert(q0), q2), 1.0/6.0));
            q2minus = q_mult(q2, q_pow(q_mult(q_invert(q1), q3), -1.0/6.0));
            
            
            // convert the times
            double t1 = (q_times[p] - q_times[0]);
            double t2 = (q_times[p+1] - q_times[0]);
            double time = round((t2 - t1) * (float)FRAME_RATE);
            
            struct Quaternion all_qs[4] = {q1, q1plus, q2minus, q2};
            
            //now slerp for each for each time t
            for (int f=0; f<time; f++) {
                double t = f/time;
                
                
                
                
                struct Quaternion w, prod;
                prod.s = 1;
                prod.v.x = 0;
                prod.v.y = 0;
                prod.v.z = 0;
                
                // the bk,n values
                double bnk_calc[4];
                bnk_calc[0] = (1 - t) * (1 - t) * (1 - t);
                bnk_calc[1] = 3 * t * (1 - t) * (1 - t);
                bnk_calc[2] = 3 * t * t * (1 - t);
                bnk_calc[3] = t * t * t;
                
                // q1, q1plus, q2minus, q2... so do that next?
                // slide 92
                for (int qidx=1; qidx<4; qidx++) {
                    w = q_log(q_mult(q_invert(all_qs[qidx-1]), all_qs[qidx]));
                    double bnk = 0;
                    
                    
                    
                    // TODO - this is coming up as zero every time
                    for (int bnk_i=qidx; bnk_i<4; bnk_i++) {
                        bnk += bnk_calc[bnk_i];
                    }
                    
                    // product from slide 82
                    if (qidx == 1) {
                        prod = q_exp(q_scalar(bnk, w));
                    } else {
                        //prod = q_mult(q_pow(w, bnk), prod);
                        prod = q_mult(prod, q_exp(q_scalar(bnk, w)));
                    }
                    
                }
                
                // q = q1 * prod
                q = q_mult(q1, prod);
                
                // add the new q to the array
                interpCurveCR[b][frame_idx][0] = q.s;
                interpCurveCR[b][frame_idx][1] = q.v.x;
                interpCurveCR[b][frame_idx][2] = q.v.y;
                interpCurveCR[b][frame_idx][3] = q.v.z;
                frame_idx++;
            }
            
            
            // error checking
            if (frame_idx > MAX_POINTS*FRAME_RATE*MAX_SECONDS) {
                printf("too many frames, cutting this short\n");
                break;
            }
        }
        numFrames = (frame_idx - 1);
    }
}

// slerp the keyframes and put in interpSlerp
void slerpKeyframes() {
    // convert the times to frames at 24 fps
    
    // for every box b
    for (int b=0; b<numBoxes; b++) {
        int frame_idx = 0; // keeps track of each frame
        // for every point p
        for (int p=0; p<numPoints; p++) {
            // convert to quaternion structs
            struct Quaternion q1;
            struct Quaternion q2;
            
            q1.s = q_points_by_box[b][p][0];
            q1.v.x = q_points_by_box[b][p][1];
            q1.v.y = q_points_by_box[b][p][2];
            q1.v.z = q_points_by_box[b][p][3];
            
            q2.s = q_points_by_box[b][p+1][0];
            q2.v.x = q_points_by_box[b][p+1][1];
            q2.v.y = q_points_by_box[b][p+1][2];
            q2.v.z = q_points_by_box[b][p+1][3];
            
            // convert the times
            float t1 = (q_times[p] - q_times[0]);
            float t2 = (q_times[p+1] - q_times[0]);
            float time = round((t2 - t1) * (float)FRAME_RATE);
            
            //now slerp for each for each time t
            for (int t=0; t<time; t++) {
                struct Quaternion q = q_slerp(q1, q2, (float)t/time);
                interpSlerp[b][frame_idx][0] = q.s;
                interpSlerp[b][frame_idx][1] = q.v.x;
                interpSlerp[b][frame_idx][2] = q.v.y;
                interpSlerp[b][frame_idx][3] = q.v.z;
                frame_idx++;
            }
            
            
            // error checking
            if (frame_idx > MAX_POINTS*FRAME_RATE*MAX_SECONDS) {
                printf("too many frames, cutting this short\n");
                break;
            }
        }
        numFrames = (frame_idx - 1);
        printf("numFrames = %d", numFrames);
    }
    
}

// loop through the slerp interpolation
void slerpLoop() {
    for (int b=0; b<numBoxes; b++) {
        struct Quaternion q;
        q.s = interpSlerp[b][cur_t][0];
        q.v.x = interpSlerp[b][cur_t][1];
        q.v.y = interpSlerp[b][cur_t][2];
        q.v.z = interpSlerp[b][cur_t][3];
        float ea, ex, ey, ez;
        q_to_axis(q, &ea, &ex, &ey, &ez);
        axis_angle[b]= ea;
        axis_x[b] = ex;
        axis_y[b] = ey;
        axis_z[b] = ez;
        
        
    }
    cur_t++;
    if (cur_t >= numFrames) {
        cur_t = 0;
    }
}

// loop through the slerp interpolation
void curveLoopBO() {
    for (int b=0; b<numBoxes; b++) {
        struct Quaternion q;
        q.s = interpCurveBO[b][cur_t][0];
        q.v.x = interpCurveBO[b][cur_t][1];
        q.v.y = interpCurveBO[b][cur_t][2];
        q.v.z = interpCurveBO[b][cur_t][3];
        float ea, ex, ey, ez;
        q_to_axis(q, &ea, &ex, &ey, &ez);
        axis_angle[b]= ea;
        axis_x[b] = ex;
        axis_y[b] = ey;
        axis_z[b] = ez;
        
        
    }
    cur_t++;
    if (cur_t >= numFrames) {
        cur_t = 0;
    }
}

// loop through the slerp interpolation
void curveLoopCR() {
    for (int b=0; b<numBoxes; b++) {
        struct Quaternion q;
        q.s = interpCurveCR[b][cur_t][0];
        q.v.x = interpCurveCR[b][cur_t][1];
        q.v.y = interpCurveCR[b][cur_t][2];
        q.v.z = interpCurveCR[b][cur_t][3];
        float ea, ex, ey, ez;
        q_to_axis(q, &ea, &ex, &ey, &ez);
        axis_angle[b]= ea;
        axis_x[b] = ex;
        axis_y[b] = ey;
        axis_z[b] = ez;
        
        
    }
    cur_t++;
    if (cur_t >= numFrames) {
        cur_t = 0;
    }
}

// loop through the keyframes 1 frame at a time
void keyframeLoop() {
    
    // for every time point t
    //for (int t=0; t<numPoints; t++) {
    // for every box b
    for (int b=0; b<numBoxes; b++) {
        struct Quaternion q;
        q.s = q_points_by_box[b][cur_t][0];
        q.v.x = q_points_by_box[b][cur_t][1];
        q.v.y = q_points_by_box[b][cur_t][2];
        q.v.z = q_points_by_box[b][cur_t][3];
        float ea, ex, ey, ez;
        q_to_axis(q, &ea, &ex, &ey, &ez);
        axis_angle[b]= ea;
        axis_x[b] = ex;
        axis_y[b] = ey;
        axis_z[b] = ez;
        
    }
    cur_t++;
    if (cur_t >= numPoints) {
        cur_t = 0;
    }
    //}
}

// keyboard callback
void myKey(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27:
            exit(1);                        // this will close the window
            break;
            
        case 's':
            // Slerp curve
            curve_type = 0;
            break;
            
        case 'c':
            // CR curve
            curve_type = 1;
            break;
            
        case 'b':
            curve_type = 2;                    // BO curve
            break;
            
    }
}

// reshape window
void myReshape(int w, int h)
{
    xMax = 100.0*w/h;
    yMax = 100.0;
    
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective (60.0, (float)w/(float)h, 0.01f, 30.0);
    
}


/////////////////////////////////// Vector and quaternion math functions ////////////////////////

double choose(double n, double k) {
    if (k == 0 || k == n)
        return 1.0;
    return choose(n-1, k-1) + choose(n-1, k);
}
double v_dot(struct Vector v1, struct Vector v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

struct Vector v_cross(struct Vector v1, struct Vector v2) {
    struct Vector v_out;
    v_out.x = v1.y * v2.z - v1.z * v2.y;
    v_out.y = v1.z * v2.x - v1.x * v2.z;
    v_out.z = v1.x * v2.y - v1.y * v2.x;
    return v_out;
}

struct Vector v_sum(struct Vector v1, struct Vector v2) {
    struct Vector v_out;
    v_out.x = v1.x + v2.x;
    v_out.y = v1.y + v2.y;
    v_out.z = v1.z + v2.z;
    return v_out;
}

struct Vector v_scalar(double s, struct Vector v) {
    v.x = s * v.x;
    v.y = s * v.y;
    v.z = s * v.z;
    return v;
}

struct Vector v_normalize(struct Vector v) {
    // normalize
    double v_norm = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    v = v_scalar(1.0/v_norm, v);
    return v;
}

// normalize a quaternion
struct Quaternion q_normalize(struct Quaternion q) {
    // normalize
    double q_norm = sqrt(q.s * q.s + q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z);
    q = q_scalar(1.0/q_norm, q);
    return q;
}

double q_magnitude(struct Quaternion q) {
    return sqrt(q.s * q.s + q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z);
}

double q_magnitude_between(struct Quaternion q1, struct Quaternion q2) {
    struct Quaternion q = q_mult(q_invert(q1), q2);
    double vlen = sqrt(q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z);
    double mag = 2 * atan2(vlen, q.s);
    return mag;
}

// quaternion dot product
double q_dot(struct Quaternion q1, struct Quaternion q2) {
    
    return q1.s * q2.s + q1.v.x * q2.v.x + q1.v.y * q2.v.y + q1.v.z * q2.v.z;
}

// this is the rotation distance between q's
// http://www.boris-belousov.net/2016/12/01/quat-dist/#using-quaternions
double q_rot_distance(struct Quaternion q1, struct Quaternion q2) {
    q2 = q_normalize(q2);
    q1 = q_normalize(q1);
    double dist = acos(2*(q_dot(q1, q2))*(q_dot(q1, q2)) - 1);
    return dist;
}

// sums
struct Quaternion q_sum(struct Quaternion q1, struct Quaternion q2) {
    struct Quaternion q;
    q.s = q1.s + q2.s;
    
    // add the otehr vals
    q.v.x = q1.v.x + q2.v.x;
    q.v.y = q1.v.y + q2.v.y;
    q.v.z = q1.v.z + q2.v.z;
    
    return q;
}

// [s1;v1][s2;v2] = [s1s2-v1.v2;s1v2+s2v1+v1Xv2]
struct Quaternion q_mult(struct Quaternion q1, struct Quaternion q2) {
    struct Quaternion q_val;
    //q1 = q_normalize(q1);
    //q2 = q_normalize(q2);
    q_val.s = q1.s * q2.s - v_dot(q1.v, q2.v);
    struct Vector v_val;
    struct Vector vect_cross = v_cross(q1.v, q2.v);
    v_val = v_sum(v_scalar(q1.s, q2.v), v_sum(v_scalar(q2.s, q1.v), vect_cross));
    q_val.v = v_val;
    
    return q_val;
}


struct Quaternion q_scalar(double s, struct Quaternion q) {
    
    q.s = s * q.s;
    q.v.x = s * q.v.x;
    q.v.y = s * q.v.y;
    q.v.z = s * q.v.z;
    return q;
}

// invert using conjugate if not a unit quaternion
struct Quaternion q_invert(struct Quaternion q) {

    struct Quaternion q_conj;
    double q_norm;
    struct Quaternion q_inv;
    
    q_conj.s = q.s;
    q_conj.v.x = -1.0 * q.v.x;
    q_conj.v.y = -1.0 * q.v.y;
    q_conj.v.z = -1.0 * q.v.z;
    
    q_norm = q.s * q.s + q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z;
    
    q_inv = q_scalar((1.0/q_norm), q_conj);
    
    return q_inv;
}

// q to euler
// from wikipedia https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void q_to_euler(struct Quaternion q, float *roll, float *pitch, float *yaw) {
    float sin_r = 2.0 * (q.s * q.v.x * q.v.y * q.v.z);
    float cos_r = 1.0 - 2.0 * (q.v.x * q.v.x + q.v.y * q.v.y);
    *roll = atan2(sin_r, cos_r);
    float sin_p = 2.0 * (q.s * q.v.y - q.v.z * q.v.x);
    if (fabs(sin_p) >= 1) {
        *pitch = M_PI/2.0;
    } else {
        *pitch = asin(sin_p);
    }
    float sin_y = 2.0 * (q.s * q.v.z + q.v.x * q.v.y);
    float cos_y = 1.0 - 2.0 * (q.v.y * q.v.y + q.v.z * q.v.z);
    *yaw = atan2(sin_y, cos_y);
}

// quaternion to axis angle
// from here http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
void q_to_axis(struct Quaternion q, float * angle, float * x, float * y, float * z) {

    q = q_normalize(q);
    *angle = 2.0 * acos(q.s) * RAD2DEG;
    
    float s = sqrt(1 - q.s * q.s);
    if (s < 0.001) {
        *x = q.v.x;
        *y = q.v.y;
        *z = q.v.z;
    } else {
        *x = q.v.x / s;
        *y = q.v.y / s;
        *z = q.v.z / s;
    }
}

// rotate a vector by a quaternion
struct Vector q_v_rotate(struct Quaternion q, struct Vector v) {
    struct Quaternion q_v;
    q_v.s = 0;
    q_v.v = v;
    struct Quaternion q_first = q_mult(q, q_v);
    struct Quaternion q_second = q_mult(q_first, q_invert(q));
    return q_second.v;
}



// slerp two quaternions
struct Quaternion q_slerp(struct Quaternion q1, struct Quaternion q2, double t) {
    double phi = acos(q_dot(q1, q2));
    struct Quaternion q1_scale = q_scalar((sin((1-t) * phi))/(sin(phi)), q1);
    struct Quaternion q2_scale = q_scalar((sin(t * phi))/(sin(phi)), q2);
    struct Quaternion q_out = q_sum(q1_scale, q2_scale);
    
    return q_out;
}

struct Quaternion q_pow(struct Quaternion q, double exp) {
    return q_exp(q_scalar(exp, q_log(q)));
}



// exp(q)
struct Quaternion q_exp(struct Quaternion q) {
    struct Quaternion q_out;
    double q_norm = sqrt(q.s * q.s + q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z);
    double ea = exp(q.s);
    double q_norm2;
    if (q_norm == 0) {
        q_norm2 = 0;
        q_out.s = cos(q_norm);
    } else {
        q_norm2 = ea * sin(q_norm)/q_norm;
        q_out.s = ea * cos(q_norm);
    }
    q_out.v.x = q_norm2 * q.v.x;
    q_out.v.y = q_norm2 * q.v.y;
    q_out.v.z = q_norm2 * q.v.z;
    return q_out;
}
                          
// log of a unit quaternion
struct Quaternion q_log(struct Quaternion q) {
    
    q = q_normalize(q);
    double q_norm = sqrt(q.s * q.s + q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z);
    double n = sqrt(q.s * q.s + q_norm);
    
    double theta = acos(q.s/n)/sqrt(q_norm);
    
    double qv = q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z;
    double theta2 = atan2(sqrt(qv), q.s);
    
    struct Vector u = v_scalar(1.0/sqrt(qv), q.v);
    struct Vector uthetha2 = v_scalar(theta2, u);
    
    struct Quaternion q_out;
    q_out.s = 0;
    q_out.v = uthetha2;
    
    return q_out;
}



/////////////////////////////////// other functions /////////////////////////////////////

void init()
{
    // probably don't need to change these?
    distance = CAM_DIST;
    eyex = -distance;
    eyey = -distance/2.0;
    eyez = distance;
    lookx = 0.0;
    looky = -5.0;
    lookz = 0.0;
    upx = 0.0;
    upy = 1.0;
    upz = 0.0;
    
    glClear(GL_COLOR_BUFFER_BIT);
    glClearDepth(1.0f);                   // Set background depth to farthest
    glEnable(GL_DEPTH_TEST);              // Enable depth testing for z-culling
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    
    
}


void
drawFloor(){
    glColor3f(0.4, 0.4, 0.4);
    glShadeModel( GL_SMOOTH );
    glPolygonMode(GL_FRONT, GL_FILL);
    glBegin(GL_QUADS);
    glVertex3f(40.0f, -1.0f, 40.0f);
    glVertex3f(-40.0f, -1.0f, 40.0f);
    glVertex3f(-40.0f, -1.0f, -40.0f);
    glVertex3f(40.0f, -1.0f, -40.0f);
    glEnd();
}




// draw the boxes
void
drawBoxes() {
    int i;
    for (i=0; i<numBoxes; i++) {
        
        // the box translation, 0 for the first, length for the rest
        if (i > 0) {
            
            glTranslatef(0.0f, -boxLength, 0.0f);
            
        }
        
        glRotatef(axis_angle[i], axis_x[i], axis_y[i], axis_z[i]);
        
        // start the box
        glBegin(GL_QUADS);
        glColor3f(1.0, 1.0, 0.0);
        int vec;
        for (vec=0; vec<12; vec+=3) {
            glVertex3f(boxVectors[i][vec], boxVectors[i][vec+1], boxVectors[i][vec+2]);
        }
        
        //top
        glColor3f(0.0, 1.0, 0.0);
        
        for (vec=12; vec<24; vec+=3) {
            glVertex3f(boxVectors[i][vec], boxVectors[i][vec+1], boxVectors[i][vec+2]);
        }
        
        // Bottom face (y = -0.5)
        glColor3f(1.0, 0.5, 0.0);
        
        for (vec=24; vec<36; vec+=3) {
            glVertex3f(boxVectors[i][vec], boxVectors[i][vec+1], boxVectors[i][vec+2]);
        }
        
        // Left face (x = -0.5)
        glColor3f(0.0, 0.0, (0.25 * i) + 0.25);
        
        for (vec=36; vec<48; vec+=3) {
            glVertex3f(boxVectors[i][vec], boxVectors[i][vec+1], boxVectors[i][vec+2]);
        }
        
        // Right face (x = 0.5)
        glColor3f((0.25 * i) + 0.25, 0.0, 0.5);
        
        for (vec=48; vec<60; vec+=3) {
            glVertex3f(boxVectors[i][vec], boxVectors[i][vec+1], boxVectors[i][vec+2]);
        }
        
        
        // Front face  (z = 0.5)
        glColor3f(1.0 - (0.25 * i), 0.0, (0.05 * i) + 0.25);
        
        for (vec=60; vec<72; vec+=3) {
            glVertex3f(boxVectors[i][vec], boxVectors[i][vec+1], boxVectors[i][vec+2]);
        }
        
        glEnd();
        
    }
}

/////////////////////////////////// file readers //////////////////////////////////////

float * initBoxVerts() {
    float boxverts[] = {
        
        // back face
        -0.5, -boxLength, -0.5,
        -0.5,  0.0, -0.5,
        0.5,  0.0, -0.5,
        0.5, -boxLength, -0.5,
        
        // top face
        0.5, 0.0, 0.5,
        0.5, 0.0, -0.5,
        -0.5, 0.0,  -0.5,
        -0.5, 0.0, 0.5,
        
        // Bottom face (y = -0.5) fixed
        0.5, -boxLength,  -0.5,
        0.5, -boxLength, 0.5,
        -0.5, -boxLength, 0.5,
        -0.5, -boxLength,  -0.5,
        
        // Left face (x = -0.5) fixed
        -0.5, -boxLength, 0.5,
        -0.5,  0.0, 0.5,
        -0.5,  0.0,  -0.5,
        -0.5, -boxLength, -0.5,
        
        // Right face (x = 0.5) fixed
        0.5, -boxLength, -0.5,
        0.5,  0.0,  -0.5,
        0.5,  0.0,  0.5,
        0.5, -boxLength, 0.5,
        
        // Front face  (z = 0.5) fixed
        0.5, -boxLength, 0.5,
        0.5,  0.0, 0.5,
        -0.5,  0.0, 0.5,
        -0.5, -boxLength, 0.5,
    };
    
    return boxverts;
}

void getFileInfo(char* filename){
    FILE *txt;
    int x, i, j;
    int time_idx = 0;
    int q_vals = 0;
    char buf[1000];
    i=0;
    txt = fopen(filename, "r");
    if (!txt){
        //file doesn't exist
        printf("The file don't exist.\n");
        return;
    } else {
        int top = 1;
        
        while (fgets(buf, 1000, txt) != NULL && i<MAX_POINTS) {
            
            //the first line says the number of cubes and length
            if (top==1){
                //numPoints = atoi(buf);
                numPoints = 0;
                
                // split the string
                char* token;
                const char s[2] = " ";
                token = strtok(buf, s);
               
                int word = 0;
                /* apply the # of cubes and length of cubes */
                while( token != NULL )
                {
                    if (word == 1) {
                        numBoxes = atoi(token);
                        // there will be 4* this many quaternion floats
                        q_vals = numBoxes*4;
                    } else if (word == 2) {
                        boxLength = atof(token);
                    }
                    
                    word++;
                    token = strtok(NULL, s);
                    if(word>2){
                        break;
                    }
                }
                top = 0;
                
                continue;
            }
            
            // for the rest, get the time and the vals
            char* token;
            const char s[2] = " ";
            token = strtok(buf, s);
            
            j = 0;
            
            while( token != NULL )
            {
                if (j == 0) {
                    q_times[time_idx] = atof(token);
                    time_idx++;
                } else {
                    q_points[i][j-1] = atof(token);
                }
                
                j++;
                token = strtok(NULL, s);
            }
            i++;
            if (i>=MAX_POINTS){
                printf("Max points exceeds %d.\n", MAX_POINTS);
            }
        }
        numPoints = i - 1;
        fclose(txt);
        
        // set up the box vertices
        float *box_v = initBoxVerts();
        for (int a=0; a<numBoxes; a++) {
            // probably don't need more boxes than this
            if(a>MAX_BOXES) {
                break;
            }
            //boxVectors[a] = initBoxVerts();
            // TODO - this is kind of silly but okay
            for (int b=0; b<72; b++) {
                boxVectors[a][b] = box_v[b];
            }
            
            // set up the quaternions by box as well
            // this will make it easier to apply parented animations
            for (int c=0; c<numPoints; c++) {
                
                
                q_points_by_box[a][c][0] = q_points[c][a*4];
                q_points_by_box[a][c][1] = q_points[c][a*4+1];
                q_points_by_box[a][c][2] = q_points[c][a*4+2];
                q_points_by_box[a][c][3] = q_points[c][a*4+3];
                
            }
        }
        
    }
    
    // set the number of frames total
    numFrames = (q_times[numPoints-1] - q_times[0])*1000/FRAME_RATE;
    
    // now interpolate the file info
    slerpKeyframes();
    curveKeyframesBO();
    curveKeyframesCR();
    
    // print all the quaternions
    printSlerp();
    printCurveBO();
    printCurveCR();
    printKeyframes();
    printCTRL();
}

// print slerp values to a file
void printSlerp() {
    FILE *f = fopen("/Users/johnalatouf/Documents/SCHOOL/advanced_anim/A1/A1/slerp.txt", "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }
    for (int t=0; t<numFrames; t++) {
        fprintf(f, "%d,", t);
        for (int b=0; b<numBoxes; b++) {
            struct Quaternion q;
            q.s = interpSlerp[b][t][0];
            q.v.x = interpSlerp[b][t][1];
            q.v.y = interpSlerp[b][t][2];
            q.v.z = interpSlerp[b][t][3];
            
            fprintf(f, "%f,%f,%f,%f,", q.s, q.v.x, q.v.y, q.v.z);
            
            
        }
        fprintf(f, "\n");
    }
    fclose(f);
}

// print curve values to a file
void printCurveBO() {
    FILE *f = fopen("/Users/johnalatouf/Documents/SCHOOL/advanced_anim/A1/A1/curveBO.txt", "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }
    for (int t=0; t<numFrames; t++) {
        fprintf(f, "%d,", t);
        for (int b=0; b<numBoxes; b++) {
            struct Quaternion q;
            q.s = interpCurveBO[b][t][0];
            q.v.x = interpCurveBO[b][t][1];
            q.v.y = interpCurveBO[b][t][2];
            q.v.z = interpCurveBO[b][t][3];
            
            fprintf(f,"%f,%f,%f,%f,", q.s, q.v.x, q.v.y, q.v.z);
            
            
        }
        fprintf(f, "\n");
    }
    fclose(f);
}

// print curve values to a file
void printCurveCR() {
    FILE *f = fopen("/Users/johnalatouf/Documents/SCHOOL/advanced_anim/A1/A1/curveCR.txt", "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }
    for (int t=0; t<numFrames; t++) {
        fprintf(f, "%d,", t);
        for (int b=0; b<numBoxes; b++) {
            struct Quaternion q;
            q.s = interpCurveCR[b][t][0];
            q.v.x = interpCurveCR[b][t][1];
            q.v.y = interpCurveCR[b][t][2];
            q.v.z = interpCurveCR[b][t][3];
            
            fprintf(f,"%f,%f,%f,%f,", q.s, q.v.x, q.v.y, q.v.z);
            
            
        }
        fprintf(f, "\n");
    }
    fclose(f);
}

// print curve values to a file
void printKeyframes() {
    FILE *f = fopen("/Users/johnalatouf/Documents/SCHOOL/advanced_anim/A1/A1/keyframes.txt", "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }
    for (int p=0; p<numPoints; p++) {
        
        double t1 = (q_times[p] - q_times[0]);
        double time = round(t1 * (float)FRAME_RATE);
        fprintf(f, "%f, ", time);
        
        for (int b=0; b<numBoxes; b++) {
            struct Quaternion q;
            q.s = q_points_by_box[b][p][0];
            q.v.x = q_points_by_box[b][p][1];
            q.v.y = q_points_by_box[b][p][2];
            q.v.z = q_points_by_box[b][p][3];
            
            fprintf(f, "%f,%f,%f,%f,", q.s, q.v.x, q.v.y, q.v.z);
            
            
        }
        fprintf(f, "\n");
    }
    fclose(f);
}

// print curve values to a file
void printCTRL() {
    FILE *f = fopen("/Users/johnalatouf/Documents/SCHOOL/advanced_anim/A1/A1/plus1.txt", "w");
    if (f == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }
    for (int t=0; t<numFrames; t++) {
        fprintf(f, "%d,", t);
        for (int b=0; b<numBoxes; b++) {
            struct Quaternion q;
            q.s = ctrlPoints[b][t][0];
            q.v.x = ctrlPoints[b][t][1];
            q.v.y = ctrlPoints[b][t][2];
            q.v.z = ctrlPoints[b][t][3];
            
            fprintf(f, "%f,%f,%f,%f,", q.s, q.v.x, q.v.y, q.v.z);
            
            
        }
        fprintf(f, "\n");
    }
    fclose(f);
}


/////////////////////////////////// main ////////////////////////////////////////////

int main(int argc, char *argv[])
{
    //get the file name from main args or else ask for it
    if (argc > 1)
    {
        getFileInfo(argv[1]);
        
        
        printf("numBoxes = %d\n", numBoxes);
        printf("boxLength = %f\n", boxLength);
        
    }
    else
    {
        // missing a file?
        printf("No file specified.");
    }
    
    
    srand((unsigned int) time(NULL));
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
    glutInitWindowSize(900, 700);
    glutCreateWindow("Assignment 1");
    init();
    glutDisplayFunc(myDisplay);
    glutIgnoreKeyRepeat(1);
    glutKeyboardFunc(myKey);
    glutReshapeFunc(myReshape);
    glutTimerFunc(1000/FRAME_RATE, myTimer, 0);
    glClearColor(0.7, 0.7, 0.7, 1.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    printf("Assignment 1: \nesc: exit program\ns: Slerp\nc: Catmull-Rom\nb: Bessel-Overhouser\n");
    
    glutMainLoop();
    
    return 0;
}
