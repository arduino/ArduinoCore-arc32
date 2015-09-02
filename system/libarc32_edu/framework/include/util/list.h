/*
 * Copyright (c) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LIST_H__
#define __LIST_H__

#include <stdbool.h>

/**
 * @defgroup list Lists
 * List management utility functions
 *
 * @ingroup util
 * @{
 */

typedef struct list {
	struct list * next;
} list_t;

typedef struct list_head_ {
	list_t * head;
	list_t * tail;
} list_head_t;

/**
 * Initialize a list structure.
 *
 * @param list the list to initialize
 */
void list_init(list_head_t * list);

/**
 * Append an element to the end of list, protected from interrupt context concurency.
 *
 * @param list the list to which element has to be added
 * @param element the element we want to add to the list.
 */
void list_add(list_head_t * list, list_t * element);

/**
 * Append an element to the begining of list, protected from interrupt context concurency.
 *
 * @param list the list to which element has to be added
 * @param element the element we want to add to the list.
 */
void list_add_head(list_head_t * list, list_t * element);

/**
 * Remove an element from the list, protected from interrupt context concurency.
 *
 * @param list the list to remove the element from
 * @param element the element to remove from the list.
 */
void list_remove(list_head_t *list, list_t * element);

/**
 * Iterate through elements of a list.
 *
 * @param lh the list we want to iterate through
 * @param cb the callback function to call for each element
 * @param param the parameter to pass to the callback in
 *              addition to the element.
 */
void list_foreach(list_head_t * lh, void(*cb)(void *, void *), void * param);

/**
 * Iterate through elements of a list, with the option
 * to remove element from the callback.
 *
 * @param lh the list we want to iterate through
 * @param cb the callback function to call for each element.
 *           if the callback returns non-zero, the element is
 *           removed from the list.
 * @param param the parameter to pass to the callback in
 *              addition to the element.
 */
void list_foreach_del(list_head_t * lh, int(*cb)(void *, void *), void * param);

/**
 * Get the first element from the list, protected from interrupt context concurency.
 *
 * @param lh the list from which the element has to be retrieved.
 * @return the element removed.
 */
list_t * list_get(list_head_t * lh);

/**
 * Check if the list is empty.
 *
 * @return 0 if not empty
 */
int list_empty(list_head_t *lh);

/**
 * Find the first item in a list matching a criteria.
 *
 * @param lh the list from which the element has to be retrieved.
 * @param cb the test function that will be applied to each item
 * @param data an opaque data passed to the test function
 *
 * @return the item or NULL
 */
list_t * list_find_first(list_head_t * lh, bool(*cb)(list_t*,void*), void *data);


/** @} */
#endif /* __LIST_H__ */
