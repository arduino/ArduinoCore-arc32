/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */

#ifndef MEMORYFREE_H
#define MEMORYFREE_H

#ifdef __cplusplus
extern "C" {
#endif

/* freeHeap: returns the size (in bytes) of unused space on the heap,
 * i.e. the number of bytes available for allocation by 'malloc()' */
int freeHeap(void);

/* freeStack: returns the size (in bytes) of remaining free space in the stack,
 * i.e. the difference between our current position in the stack, and the end
 * of usable stack space.
 *
 * NOTE: This function will return a negative number to indicate a stack
 * overflow, i.e. a return value of -20 means you have overrun the allocated
 * stack area by 20 bytes. */
int freeStack(void);

/* freeMemory: returns the combined free memory in both the stack and heap,
 * except in the case where a stack overflow has occurred (i.e. freeStack
 * returns a negative number). In this case, only the amount of free heap
 * space will be returned. */
int freeMemory(void);

#ifdef __cplusplus
}
#endif

#endif
