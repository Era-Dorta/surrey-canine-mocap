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

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "BVHFormat.h"

BVHFormat::BVHFormat() :
			MocapData() {
}

BVHFormat::BVHFormat(MocapHeader *header) :
			MocapData(header) {
}

void BVHFormat::EnlargeNodeList() {
	nodelist.push_back(NULL);
}

bool BVHFormat::ImportData(const char *filename) {
	int read, i, j, where;
	int pos[8];      // Used to determine the position of the next char to write
	char line[8][40]; // Used to store the attribute and the corresponding value
	char buffer[4097];
	int section = 0;     // Indicates which section is currently being processed
	Node *curnode = 0; // Used to indicate the current node that is being processed
	int index, channels = 0;
	bool endsite = false;

	header->callib = 1.0f;
	header->scalefactor = 1.0f;
	header->noofsegments = 0;
	header->noofframes = 0;
	header->datarate = 0;

	xpos = 1;
	ypos = 2;
	zpos = 0;

	header->euler->at(0).set(0, 0, 1);
	header->euler->at(1).set(1, 0, 0);
	header->euler->at(2).set(0, 1, 0);

	//TODO They had this calibration to make every model smaller
	header->callib = 0.3f;
	header->inv_callib = 1.0 / header->callib;
	header->degrees = true;
	header->scalefactor = 1.0f;

	FILE *file = fopen(filename, "rb");
	if (file) {
		// Process the "Hierarchy" section of the file
		read = fread(buffer, 1, 4096, file);
		buffer[read] = '\0';
		i = strstrEx(buffer, "HIERARCHY");
		i += strstrEx(buffer + i, char(10));
		while (buffer[++i] < 32)
			;

		where = pos[0] = pos[1] = pos[2] = pos[3] = pos[4] = pos[5] = pos[6] =
				pos[7] = 0;
		// Process each line in the header
		while (read) {
			while (i < read) {
				if ((buffer[i] == char(10) && pos[0])
						|| (section == 2 && where == 3)) {
					// Process line
					line[7][pos[7]] = line[6][pos[6]] = line[5][pos[5]] =
							line[4][pos[4]] = line[3][pos[3]] =
									line[2][pos[2]] = line[1][pos[1]] =
											line[0][pos[0]] = '\0';
					if (!section) {
						// Process Hierarchy
						if (strcompEx(line[0], "ROOT")) {
							if (root) {
								strcpy(error,
										"BVH file contains more than one skeleton which is currently unsupported");
								fclose(file);
								return false;
							} else {
								EnlargeNodeList();
								root = nodelist[header->noofsegments++] =
										new Node();
								root->name = std::string(line[1]);
								curnode = root;
							}
						} else if (strcompEx(line[0], "JOINT")) {
							curnode->increase_no_children();
							EnlargeNodeList();
							curnode->children[curnode->noofchildren - 1] =
									nodelist[header->noofsegments++] =
											new Node();
							curnode->children[curnode->noofchildren - 1]->parent =
									curnode;
							curnode = curnode->children[curnode->noofchildren
									- 1];

							curnode->name = std::string(line[1]);
						} else if (strcompEx(line[0], "OFFSET")) {
							float x, y, z, rx, ry, rz;
							x = (float) atof(line[1]) * header->callib;
							y = (float) atof(line[2]) * header->callib;
							z = (float) atof(line[3]) * header->callib;
							rx = ry = rz = 0.0f;
							if (!endsite) {
								curnode->setup_offset(x, y, z);
								if (curnode != root
										&& (curnode->parent->length[0] == 0.0f
												&& curnode->parent->length[1]
														== 0.0f
												&& curnode->parent->length[2]
														== 0.0f)) {
									curnode->parent->setup_euler(rx, ry, rz);
									curnode->parent->length.set(x, y, z);
								}
							} else {
								curnode->length.set(x, y, z);
							}
						} else if (strcompEx(line[0], "CHANNELS") && !endsite) {
							channels += atoi(line[1]);
							curnode->noofchannels = atoi(line[1]);
							int d = 2;
							while (line[d] && d < 8) {
								if ((line[d][0] & 0xdf) == 'X') {
									if ((line[d][1] & 0xdf) == 'R') {
										curnode->DOFs |= XROT;
									} else if ((line[d][1] & 0xdf) == 'P')
										curnode->DOFs |= XTRA;
								} else if ((line[d][0] & 0xdf) == 'Y') {
									if ((line[d][1] & 0xdf) == 'R') {
										curnode->DOFs |= YROT;
									} else if ((line[d][1] & 0xdf) == 'P')
										curnode->DOFs |= YTRA;
								} else if ((line[d][0] & 0xdf) == 'Z') {
									if ((line[d][1] & 0xdf) == 'R') {
										curnode->DOFs |= ZROT;
									} else if ((line[d][1] & 0xdf) == 'P')
										curnode->DOFs |= ZTRA;
								}
								++d;
							}
						} else if (strcompEx(line[0], "END")
								&& strcompEx(line[1], "SITE"))
							endsite = true;
						else if (line[0][0] == '}') {
							if (endsite)
								endsite = false;
							else
								curnode = curnode->parent;
						} else if (strcompEx(line[0], "MOTION")) {
							++section;
						}
					} else if (section == 1) {
						// Process Motion
						if (strcompEx(line[0], "FRAMES:")) {
							header->noofframes = atoi(line[1]);
							for (int i = 0; i < header->noofsegments; ++i)
								nodelist[i]->setup_frames(header->noofframes);
							header->currentframe = 0;
						} else if (strcompEx(line[0], "FRAME")
								&& strcompEx(line[1], "TIME:")) {
							header->frametime = atof(line[2]);
							header->datarate = (int) (1 / (atof(line[2])));
							if ((int) (0.49 + (1 / atof(line[2])))
									> header->datarate)
								++header->datarate;
						}
						if (header->datarate && header->noofframes) {
							++section;
							curnode = root;
							index = 0;
							endsite = false;
						}
					} else {
						//Process DOFs
						if (header->currentframe < header->noofframes) {
							float v0, v1, v2;
							v0 = (float) atof(line[0]);
							v1 = (float) atof(line[1]);
							v2 = (float) atof(line[2]);
							if (curnode->DOFs == 231) {
								if (!endsite) {
									curnode->froset->at(header->currentframe).set(
											v0 * header->callib,
											v1 * header->callib,
											v2 * header->callib);
									endsite = true;
								} else {
									curnode->freuler->at(header->currentframe).set(
											v0, v1, v2);
									curnode->scale[header->currentframe] = 1.0f;
									curnode = nodelist[++index];
									endsite = false;
								}
							} else {
								curnode->froset->at(header->currentframe)[0] =
										curnode->froset->at(
												header->currentframe)[1] =
												curnode->froset->at(
														header->currentframe)[2] =
														0.0f;
								curnode->freuler->at(header->currentframe).set(
										v0, v1, v2);
								curnode->scale[header->currentframe] = 1.0f;

								if (index + 1 < header->noofsegments)
									curnode = nodelist[++index];
								else {
									++header->currentframe;
									curnode = nodelist[index = 0];
								}
							}
						} else
							++section;
					}

					if (section != 2) {
						// Move onto the next line and clear current line information
						j = strstrEx(buffer + i, char(10));
						if (j == -1) {
							if (buffer[4095] != 10) {
								read = fread(buffer, 1, 4096, file);
								i = strstrEx(buffer, char(10));
							} else {
								read = fread(buffer, 1, 4096, file);
								i = 0;
							}
							buffer[4096] = '\0';
						} else
							i += j;
					}
					where = pos[0] = pos[1] = pos[2] = pos[3] = pos[4] =
							pos[5] = pos[6] = pos[7] = 0;
				}

				if (buffer[i] > 44 && buffer[i] < 126)
					line[where][pos[where]++] = buffer[i++];
				else if ((buffer[i] == 32 || buffer[i] == 9)
						&& pos[where] > 0) {
					++where;
					++i;
				} else
					++i;
			}
			read = fread(buffer, 1, 4096, file);
			buffer[4096] = '\0';
			i = 0;
		}
		fclose(file);
		return true;
	} else {
		strcpy(error, "Cannot Open File");
		return false;
	}
}

bool BVHFormat::ExportData(const char* filename) {
	std::ofstream out_file;
	out_file.open(filename);
	//Set so that all float numbers are written with 6 decimals
	out_file.precision(6);
	out_file.setf(std::ios::fixed, std::ios::floatfield);

	ExportHierarchy(out_file);

	ExportMotion(out_file);

	return true;
}

void BVHFormat::ExportDataJoint(std::ofstream& out_file, Node* parent,
		Node* joint, int tabs, bool print_parent) {
	std::string tabs_str;
	for (int i = 0; i < tabs; i++) {
		tabs_str += "\t";
	}
	tabs++;

	if (print_parent) {
		out_file << tabs_str << "OFFSET "
				<< parent->offset[0] * header->inv_callib << " "
				<< parent->offset[1] * header->inv_callib << " "
				<< parent->offset[2] * header->inv_callib << endl;
		out_file << tabs_str << "CHANNELS " << parent->noofchannels;
		if (parent->noofchannels == 3) {
			out_file << " Xrotation Yrotation Zrotation" << endl;
		} else {
			out_file
					<< " Xposition Yposition Zposition Xrotation Yrotation Zrotation"
					<< endl;
		}
	}
	out_file << tabs_str << "JOINT " << joint->name << endl;
	out_file << tabs_str << "{" << endl;
	tabs_str += "\t";

	bool print_data = true;
	for (int i = 0; i < joint->noofchildren; i++) {
		ExportDataJoint(out_file, joint, joint->children[i], tabs, print_data);
		print_data = false;
	}

	if (joint->noofchildren == 0) {
		ExportEndSite(out_file, joint, tabs);
	}
	tabs_str.erase(tabs_str.length() - 1);
	out_file << tabs_str << "}" << endl;
}

void BVHFormat::ExportEndSite(std::ofstream& out_file, Node* joint, int tabs) {
	std::string tabs_str;
	for (int i = 0; i < tabs; i++) {
		tabs_str += "\t";
	}
	out_file << tabs_str << "OFFSET " << joint->offset[0] * header->inv_callib
			<< " " << joint->offset[1] * header->inv_callib << " "
			<< joint->offset[2] * header->inv_callib << endl;
	out_file << tabs_str << "CHANNELS " << joint->noofchannels;
	if (joint->noofchannels == 3) {
		out_file << " Xrotation Yrotation Zrotation" << endl;
	} else {
		out_file
				<< " Xposition Yposition Zposition Xrotation Yrotation Zrotation"
				<< endl;
	}
	out_file << tabs_str << "End Site" << endl;
	out_file << tabs_str << "{" << endl;
	out_file << tabs_str + "\t" << "OFFSET "
			<< joint->length[0] * header->inv_callib << " "
			<< joint->length[1] * header->inv_callib << " "
			<< joint->length[2] * header->inv_callib << endl;
	out_file << tabs_str << "}" << endl;
}

void BVHFormat::ExportHierarchy(std::ofstream& out_file) {
	out_file << "HIERARCHY" << endl;
	out_file << "ROOT " << root->name << endl;
	out_file << "{" << endl;
	bool print_data = true;
	for (int i = 0; i < root->noofchildren; i++) {
		ExportDataJoint(out_file, root, root->children[i], 1, print_data);
		print_data = false;
	}
	if (root->noofchildren == 0) {
		ExportEndSite(out_file, root, 1);
	}
	out_file << "}" << endl;
}

void BVHFormat::ExportMotion(std::ofstream& out_file) {
	out_file << "MOTION" << endl;
	out_file << "Frames: " << header->noofframes << endl;
	out_file << "Frame Time: " << header->frametime << endl;

	for (int i = 0; i < header->noofframes; i++) {
		int j;
		//Root node is the only one with per frame offset and 6 channels
		out_file << nodelist[0]->froset->at(i)[0] * header->inv_callib << " "
				<< nodelist[0]->froset->at(i)[1] * header->inv_callib << " "
				<< nodelist[0]->froset->at(i)[2] * header->inv_callib << " ";

		//All the other nodes is just angles
		for (j = 0; j < header->noofsegments - 1; j++) {
			out_file << nodelist[j]->freuler->at(i)[0] << " "
					<< nodelist[j]->freuler->at(i)[1] << " "
					<< nodelist[j]->freuler->at(i)[2] << " ";
		}
		//Last line substitute space for with line feed
		out_file << nodelist[j]->freuler->at(i)[0] << " "
				<< nodelist[j]->freuler->at(i)[1] << " "
				<< nodelist[j]->freuler->at(i)[2] << endl;
	}
}
