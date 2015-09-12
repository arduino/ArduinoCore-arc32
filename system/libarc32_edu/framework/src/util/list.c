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

#include "os/os.h"
#include "util/list.h"

void list_init(list_head_t * list) {
    list->head = list->tail = NULL;
}

void list_add_head(list_head_t * list, list_t * element) {
    uint32_t saved = interrupt_lock();
    element->next = list->head;
    list->head = element;
    if (element->next == NULL) {
        list->tail = element;
    }
    interrupt_unlock(saved);
}


void list_add(list_head_t * list, list_t * element) {
    uint32_t saved = interrupt_lock();
    if (list->head == NULL) {
        list->head = list->tail = element;
    } else {
        list->tail->next = element;
        list->tail = element;
    }
    element->next = NULL;
    interrupt_unlock(saved);
}

void list_remove(list_head_t *list, list_t * element) {
    uint32_t saved = interrupt_lock();
    list_t * l = list->head;
    if (l == NULL) {
        // List empty, return
        goto exit;
    }
    /* remove first? */
    if (element == l) {
        list->head = l->next;
        if (list->head == NULL) {
            list->tail = NULL;
        }
    } else {
        // Find element in the list
        for (; (l->next) && (l->next != element); (l = l->next));

        if (l->next != NULL) {
            // remove element
            if (list->tail == l->next) {
                list->tail = l;
            }
            l->next = l->next->next;
        }
    }
exit:
    interrupt_unlock(saved);
}

void list_foreach(list_head_t * lh, void(*cb)(void *, void *), void * param) {
    list_t * l = lh->head;
    while(l) {
        cb(l, param);
        l = l->next;
    }
}

void list_foreach_del(list_head_t * lh, int(*cb)(void *, void *), void * param) {
    list_t * l = lh->head;
    list_t * prev = lh->head;
    while(l) {
        list_t * tmp;
        tmp = l->next;
        if (cb(l, param)) {
            if (l == lh->head) {
                lh->head = tmp;
                prev = tmp;
                if (lh->tail == l) {
                    lh->tail = NULL;
                }
                l = tmp;
            } else {
                prev->next = tmp;
                if (lh->tail == l) {
                    lh->tail = prev;
                }
                l = prev->next;
            }
        } else {
            prev = l;
            l = l->next;
        }
    }
}

list_t * list_get(list_head_t *lh) {
    uint32_t saved = interrupt_lock();
    list_t * l = lh->head;
    if (l != NULL) {
        lh->head = l->next;
        if (lh->head == NULL) {
            lh->tail = NULL;
        }
    }
    interrupt_unlock(saved);
    return l;
}

int list_empty(list_head_t *lh) {
    return (lh->head == NULL);
}

list_t * list_find_first(list_head_t * lh, bool(*cb)(list_t*,void*), void *data)
{
    list_t *l = lh->head;
    for (; l && !cb(l,data); l = l->next);
    return l;
}
