/*************************************************************************

 Graphics Research Group - Department of Computer Science
 University of Sheffield, UK
 Michael Meredith
 Copyright (c) 2000
 All Rights Reserved.


 Permission is hereby granted, free of charge, to use this software
 and its documentation without restriction, including without limitation
 the rights to use, copy, modify, merge and publish, subject to the following
 conditions:

 1. The code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Any modifications must be clearly marked as such.

 3. Original authors' names are not deleted.

 4. The authors' names are not used to endorse or promote products
 derived from this software without specific prior written
 permission from the author.

 5. This software is used in a research, non-profit capacity only and not
 for commercial or profit applications unless specific prior written
 permission from the author is given.


 THE UNIVERSITY OF SHEFFIELD AND THE CONTRIBUTORS TO THIS WORK
 DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING
 ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO EVENT
 SHALL THE UNIVERSITY OF SHEFFIELD NOR THE CONTRIBUTORS BE LIABLE
 FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
 AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
 ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF
 THIS SOFTWARE.

 *************************************************************************/

#ifndef __extras_h__
#define __extras_h__
#include <stdlib.h>

#define XROT 1
#define YROT 2
#define ZROT 4
#define XTRA 32
#define YTRA 64
#define ZTRA 128
#define PI 3.141592

typedef unsigned char BYTE;

struct NODE {
		char *name;
		float length[3];    // length of segment
		float offset[3]; // Transitional offset with respect to the end of the partent link
		float euler[3];     // Rotation
		float colour[3];
		int noofchildren;
		NODE **children;    // Array of pointers to child nodes
		NODE *parent;       // Back pointer to parent node
		float **froset;     // Array of offsets for each frame
		float **freuler;    // Array of angles for each frame
		float *scale;       // Array of scalefactors for each frame
		BYTE DOFs;          // Used to determine what DOFs the segment has
};

struct MOCAPHEADER {
		// Assumes that all angles are in degrees if not then they need to be converted
		int noofsegments;       // Number of body segments
		long noofframes;     // Number of frames
		int datarate;         // Number of frames per second
		int euler[3][3];      // Specifies how the euler angle is defined
		float callib; // Scale factor for converting current translational units into meters
		bool degrees;         // Are the rotational measurements in degrees
		float scalefactor;    // Global Scale factor
		long currentframe;    // Stores the currentframe to render
};

int strstrEx(const char *string, const char *strCharSet);
int strstrEx(const char *string, char strChar);
bool strcompEx(const char *string, const char *strCharSet);
float sqr(float a);

inline void SetupChildren(NODE* seg, int children) {
	seg->noofchildren = children;
	if (children)
		seg->children = (NODE**) malloc(sizeof(NODE*) * children);
	else
		seg->children = 0;
}

inline void SetupOffset(NODE* seg, float x = 0.0f, float y = 0.0f, float z =
		0.0f) {
	seg->offset[0] = x;
	seg->offset[1] = y;
	seg->offset[2] = z;
}

inline void SetupEuler(NODE* seg, float r1 = 0.0f, float r2 = 0.0f, float r3 =
		0.0f) {
	seg->euler[0] = r1;
	seg->euler[1] = r2;
	seg->euler[2] = r3;
}

inline void SetupColour(NODE* seg, float r = 0.0f, float g = 0.0f, float b =
		0.0f) {
	seg->colour[0] = r;
	seg->colour[1] = g;
	seg->colour[2] = b;
}

inline void SetupFrames(NODE* seg, long frames) {
	seg->froset = (float**) malloc(sizeof(float*) * frames);
	seg->freuler = (float**) malloc(sizeof(float*) * frames);
	seg->scale = (float*) malloc(sizeof(float) * frames);
	for (long i = 0; i < frames; ++i) {
		seg->froset[i] = (float*) malloc(sizeof(float*) * 3);
		seg->freuler[i] = (float*) malloc(sizeof(float*) * 3);
	}
}

inline float sqr(float a) {
	return a * a;
}

#endif
