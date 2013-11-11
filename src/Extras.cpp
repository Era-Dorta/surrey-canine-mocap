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

#include "Extras.h"

int strstrEx(const char *string, const char *strCharSet) {
	// Like strstr but not case sensitve and returns the offset within the string of the start of strCharSet
	// Assumes that strCharSet is already presented in upper case
	int i = 0, j = 0, k = 0;
	if (!strCharSet[0])
		return -1;
	while (string[i]) {
		while (((string[i] > 96 && string[i] < 123) ?
				string[i] & 0xdf : string[i]) != strCharSet[0] && string[i++])
			;
		j = ++i;
		while (((string[j] > 96 && string[j] < 123) ?
				string[j] & 0xdf : string[j]) == strCharSet[++k] && string[j++])
			;
		if (strCharSet[k])
			k = 0;
		else
			return i - 1;
	}
	return -1;
}

int strstrEx(const char *string, char strChar) {
	// Like strchr but not case sensitve and returns the offset within the string of the start of strCharSet
	// Assumes that strChar is already presented in upper case
	int i = 0;
	if (!strChar)
		return -1;
	while (string[i] && (string[i++] & 0xdf) != strChar)
		;
	if (string[i])
		return i - 1;
	else
		return -1;
}

bool strcompEx(const char *string, const char* strCharSet) {
	int i = 0;
	while (string[i] && strCharSet[i] && (string[i] & 0xdf) == strCharSet[i++])
		;
	if (!string[i] && !strCharSet[i])
		return true;
	else
		return false;
}
