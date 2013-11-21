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

inline std::ostream& operator<<(std::ostream &out, const osg::Vec3 &vector) {
	out << vector.x() << " " << vector.y() << " " << vector.z();
	return out;
}

BVHFormat::BVHFormat() :
			MocapData() {
}

bool BVHFormat::import_data(const char *filename) {
	int read, i, j, where;
	int pos[8];      // Used to determine the position of the next char to write
	char line[8][40]; // Used to store the attribute and the corresponding value
	char buffer[4097];
	int section = 0;     // Indicates which section is currently being processed
	Node *curnode = 0; // Used to indicate the current node that is being processed
	int index = 0, channels = 0;
	bool endsite = false;

	reset_state();

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
							if (root.get()) {
								strcpy(error,
										"BVH file contains more than one skeleton which is currently unsupported");
								fclose(file);
								return false;
							} else {
								root = NodePtr(new Node);
								nodelist.push_back(root.get());
								header.noofsegments++;
								root->name = std::string(line[1]);
								curnode = root.get();
							}
						} else if (strcompEx(line[0], "JOINT")) {
							curnode->increase_no_children();
							curnode = curnode->get_last_child();
							nodelist.push_back(curnode);
							header.noofsegments++;
							curnode->name = std::string(line[1]);
						} else if (strcompEx(line[0], "OFFSET")) {
							float x, y, z;
							x = (float) atof(line[1]) * header.callib;
							y = (float) atof(line[2]) * header.callib;
							z = (float) atof(line[3]) * header.callib;
							if (!endsite) {
								curnode->setup_offset(x, y, z);
								if (curnode != root.get()
										&& (curnode->parent->length[0] == 0.0f
												&& curnode->parent->length[1]
														== 0.0f
												&& curnode->parent->length[2]
														== 0.0f)) {
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
							header.noofframes = atoi(line[1]);
							for (int i = 0; i < header.noofsegments; ++i)
								nodelist[i]->resize_frame_no(header.noofframes);
							header.currentframe = 0;
						} else if (strcompEx(line[0], "FRAME")
								&& strcompEx(line[1], "TIME:")) {
							header.frametime = atof(line[2]);
							header.datarate = (int) (1 / (atof(line[2])));
							if ((int) (0.49 + (1 / atof(line[2])))
									> header.datarate)
								++header.datarate;
						}
						if (header.datarate && header.noofframes) {
							++section;
							curnode = root.get();
							index = 0;
							endsite = false;
						}
					} else {
						//Process DOFs
						if (header.currentframe < header.noofframes) {
							osg::Vec3 v;
							//float v0, v1, v2;
							v.set((float) atof(line[0]), (float) atof(line[1]),
									(float) atof(line[2]));
							if (curnode->DOFs == 231) {
								if (!endsite) {
									curnode->froset->at(header.currentframe).set(
											v * header.callib);
									endsite = true;
								} else {
									curnode->freuler->at(header.currentframe).set(
											degrees_to_radians(v));
									curnode = nodelist[++index];
									endsite = false;
								}
							} else {
								curnode->froset->at(header.currentframe).set(
										0.0f, 0.0f, 0.0f);
								curnode->freuler->at(header.currentframe).set(
										degrees_to_radians(v));

								if (index + 1 < header.noofsegments)
									curnode = nodelist[++index];
								else {
									++header.currentframe;
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

bool BVHFormat::export_data(const char* filename) {
	std::ofstream out_file;
	out_file.open(filename);

	if (out_file.is_open()) {
		try {
			//Set so that all float numbers are written with 6 decimals
			out_file.precision(6);
			out_file.setf(std::ios::fixed, std::ios::floatfield);

			export_hierarchy(out_file);

			export_motion(out_file);
		} catch (...) {
			cout << "Error when saving BVH file" << endl;
			out_file.close();
			throw;
		}
		out_file.close();
		return true;
	} else {
		cout << "Could not open file to save BVH";
		return false;
	}
}

void BVHFormat::export_data_joint(std::ofstream& out_file, Node* parent,
		Node* joint, std::string& tabs_str, bool print_parent) {
	if (print_parent) {
		out_file << tabs_str << "OFFSET " << parent->offset * header.inv_callib
				<< endl;
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
	for (unsigned int i = 0; i < joint->get_num_children(); i++) {
		export_data_joint(out_file, joint, joint->children[i].get(), tabs_str,
				print_data);
		print_data = false;
	}

	if (joint->get_num_children() == 0) {
		export_end_site(out_file, joint, tabs_str);
	}
	tabs_str.erase(tabs_str.length() - 1);
	out_file << tabs_str << "}" << endl;
}

void BVHFormat::export_end_site(std::ofstream& out_file, Node* joint,
		std::string& tabs_str) {

	out_file << tabs_str << "OFFSET " << joint->offset * header.inv_callib
			<< endl;
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
			<< joint->length * header.inv_callib << endl;
	out_file << tabs_str << "}" << endl;
}

void BVHFormat::export_hierarchy(std::ofstream& out_file) {
	out_file << "HIERARCHY" << endl;
	out_file << "ROOT " << root->name << endl;
	out_file << "{" << endl;
	bool print_data = true;
	std::string tabs("\t");
	for (unsigned int i = 0; i < root->get_num_children(); i++) {
		export_data_joint(out_file, root.get(), root->children[i].get(), tabs,
				print_data);
		print_data = false;
	}
	if (root->get_num_children() == 0) {
		export_end_site(out_file, root.get(), tabs);
	}
	out_file << "}" << endl;
}

void BVHFormat::export_motion(std::ofstream& out_file) {
	out_file << "MOTION" << endl;
	out_file << "Frames: " << header.noofframes << endl;
	out_file << "Frame Time: " << header.frametime << endl;

	for (int i = 0; i < header.noofframes; i++) {
		int j;
		//Root node is the only one with per frame offset and 6 channels
		out_file << nodelist[0]->froset->at(i) * header.inv_callib << " ";

		//All the other nodes is just angles
		for (j = 0; j < header.noofsegments - 1; j++) {
			out_file << radians_to_degrees(nodelist[j]->freuler->at(i)) << " ";
		}
		//Last line substitute space for with line feed
		out_file << radians_to_degrees(nodelist[j]->freuler->at(i)) << endl;
	}
}

osg::Vec3 BVHFormat::radians_to_degrees(osg::Vec3& v) {
	osg::Vec3 res;
	res[0] = osg::RadiansToDegrees(v[0]);
	res[1] = osg::RadiansToDegrees(v[1]);
	res[2] = osg::RadiansToDegrees(v[2]);
	return res;
}

osg::Vec3 BVHFormat::degrees_to_radians(osg::Vec3& v) {
	osg::Vec3 res;
	res[0] = osg::DegreesToRadians(v[0]);
	res[1] = osg::DegreesToRadians(v[1]);
	res[2] = osg::DegreesToRadians(v[2]);
	return res;
}
