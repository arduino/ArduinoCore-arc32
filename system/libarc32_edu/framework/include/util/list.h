#ifndef __LIST_H__
#define __LIST_H__

#include <stdbool.h>

typedef struct list {
	struct list * next;
} list_t;

typedef struct list_head_ {
	list_t * head;
	list_t * tail;
} list_head_t;

/**
 * initialize a list structure
 *
 * \param list the list to initialize
 */
void list_init(list_head_t * list);

/**
 * append an element to the end of list.
 *
 * \param list the list to which element has to be added
 * \param element the element we want to add to the list.
 */
void list_add(list_head_t * list, list_t * element);

/**
 * append an element to the begining of list.
 *
 * \param list the list to which element has to be added
 * \param element the element we want to add to the list.
 */
void list_add_head(list_head_t * list, list_t * element);

/**
 * remove an element from the list.
 *
 * \param list the list to remove the element from
 * \param element the element to remove from the list.
 */
void list_remove(list_head_t *list, list_t * element);

/**
 * Iterate through elements of a list.
 *
 * \param lh the list we want to iterate through
 * \param cb the callback function to call for each element
 * \param param the parameter to pass to the callback in
 *              addition to the element.
 */
void list_foreach(list_head_t * lh, void(*cb)(void *, void *), void * param);

/**
 * Iterate through elements of a list, with the option
 * to remove element from the callback.
 *
 * \param lh the list we want to iterate through
 * \param cb the callback function to call for each element.
 *           if the callback returns non-zero, the element is
 *           removed from the list.
 * \param param the parameter to pass to the callback in
 *              addition to the element.
 */
void list_foreach_del(list_head_t * lh, int(*cb)(void *, void *), void * param);

/**
 * Get the first element from the list.
 *
 * \param lh the list from which the element has to be retrieved.
 * \return the element removed.
 */
list_t * list_get(list_head_t * lh);

/**
 * Check if the list is empty
 *
 * \return 0 if not empty
 */
int list_empty(list_head_t *lh);

/**
 * Find the first item in a list matching a criteria.
 *
 * \param cb the test function that will be applied to each item
 * \param data an opaque data passed to the test function
 *
 * \return the item or NULL
 */
list_t * list_find_first(list_head_t * lh, bool(*cb)(list_t*,void*), void *data);

#endif /* __LIST_H__ */
