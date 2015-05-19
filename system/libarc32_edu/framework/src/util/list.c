#include "os/os.h"
#include "util/list.h"


void list_init(list_head_t * list) {
    list->head = list->tail = NULL;
}

void list_add_head(list_head_t * list, list_t * element) {
    if (list->head == NULL) {
        list->head = list->tail = element;
    } else {
        element->next = list->head;
        list->head = element;
    }
    element->next = NULL;
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
    list_t * l = list->head;
    /* remove first? */
    if (element == l) {
        list->head = l->next;
        if (list->head == NULL) {
            list->tail = NULL;
        }
    } else {
        list_t * prev = l;
        while (l) {
            if (l == element) {
                prev->next = l->next;
                if (list->tail == l) {
                    list->tail = prev;
                }
            }
            prev = l;
            l = l->next;
        }
    }
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
    }
    interrupt_unlock(saved);
    return l;
}

int list_empty(list_head_t *lh) {
    return (lh->head == NULL);
}

list_t * list_find_first(list_head_t * lh, bool(*cb)(list_t*,void*), void *data)
{
	list_t *result = NULL;
	list_t *item = lh->head;
	while (item && !result) {
		if (cb(item,data)) {
			result = item;
		}
		item = item->next;
	}
	return result;
}
