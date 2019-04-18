
Description:

This program uses OpenGL to animated a linked pendulum with data specified in uniform and 
non-uniform .dat files.

-----------------------------------------------------------------------------------------

Requirements:

The program was compiled on a Mac using the following c libraries:
<stdlib.h>
<string.h>
<time.h>
<math.h>
<GLUT/glut.h>
<OpenGL/gl.h>
<stdio.h>

On Windows, the GLUT and OpenGL libraries should be changed to Windows specific libraries.
GCC is required to compile the script.

-----------------------------------------------------------------------------------------

How it works:
See the description.pdf file for a detailed description of the interpolation math.

The program reads a .dat file specified on the command line and uses it to draw a set of 
boxes, parented to the box above, oriented in the top centre. The points in the .dat file
are interpolated into quaternion animation frames that are converted to rotation angles 
and applied to the boxes.

-----------------------------------------------------------------------------------------

Usage Instructions:

To compile using command line in Mac or Linux environment:
gcc -std=c99 -o a1.out main.c -framework OpenGL -framework GLUT

To run from command line:
./a1.out [.DAT FILE PATH]

To switch interpolations with keys:
s: Slerp (this runs by default)
c: Catmull Rom
b: Bessel Overhauser

To exit:
esc

-----------------------------------------------------------------------------------------

