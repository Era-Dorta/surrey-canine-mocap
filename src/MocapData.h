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

#ifndef __mocapdata_h__
#define __mocapdata_h__
#include "Extras.h"

#include <iostream>

using std::cout;
using std::endl;

class MocapData {
	public:
		MocapData();
		MocapData(MOCAPHEADER *header);
		virtual ~MocapData();

		void SetHeader(MOCAPHEADER *header); // Sets up a pointer to the header structure
		virtual bool ImportData(const char* filename) = 0; // Starts the import of the BVH file
		virtual bool ExportData(const char* filename) = 0; // Starts the import of the BVH file
		NODE* GetRootNode();                      // Returns the root node
		NODE** GetNodeList();       // Returns an array that holds all the nodes

		const char* GetError();

	protected:
		NODE *root, **nodelist;
		MOCAPHEADER* header;
		int xpos, ypos, zpos;

		char error[255];          // Used to store any errors that occured
	private:
		void free_node_memory(struct NODE* to_delete);
		void delete_recursive(struct NODE* to_delete);
};

#endif
