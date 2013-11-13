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

#ifndef __bvhformat_h__
#define __bvhformat_h__
#include "MocapData.h"
#include "Extras.h"
#include <fstream>
#include <iostream>
using std::cout;
using std::endl;

class BVHFormat: public MocapData {
	public:
		BVHFormat();
		BVHFormat(MocapHeader *header);

		bool ImportData(const char* filename); // Starts the import of the BVH file
		bool ExportData(const char* filename); // Starts the export of the BVH file

	private:
		//Recursive method that writes a node information to a file BVH file
		void ExportDataJoint(std::ofstream& out_file, Node* parent, Node* joint,
				int tabs, bool print_parent);

		//Method that writes a node information to a BVH file
		void ExportEndSite(std::ofstream& out_file, Node* joint, int tabs);

		//Method that writes the hierarchy part to a BVH file
		void ExportHierarchy(std::ofstream& out_file);

		//Method that writes the motion part to a BVH file
		void ExportMotion(std::ofstream& out_file);

		void EnlargeNodeList();
};

#endif
