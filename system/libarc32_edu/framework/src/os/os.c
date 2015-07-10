#include "cfw/cfw.h"
#include "os/os.h"

/*************************    MEMORY   *************************/

#ifdef TRACK_ALLOCS
int alloc_count = 0;
#endif

void * cfw_alloc(int size, OS_ERR_TYPE * err) {
    void * ptr;
    unsigned int flags = interrupt_lock();
    ptr = malloc(size+sizeof(void*));
    (*(int*) ptr) = size;
#ifdef TRACK_ALLOCS
    alloc_count++;
#endif
    interrupt_unlock(flags);
    return ptr;
}

void cfw_free(void * ptr, OS_ERR_TYPE * err) {
    int flags = interrupt_lock();
#ifdef TRACK_ALLOCS
    alloc_count--;
#endif
    free(ptr);
    interrupt_unlock(flags);
}

void * balloc(uint32_t size, OS_ERR_TYPE *err) {
	return cfw_alloc(size, err);
}

OS_ERR_TYPE bfree(void *ptr) {
	cfw_free(ptr, NULL);
	return E_OS_OK;
}

/*************************    QUEUES   *************************/

typedef struct queue_ {
    list_head_t lh;
    int count;
    int used;
} q_t;

q_t q_pool[10];

void queue_put(void *queue, void *msg) {
    q_t * q = (q_t*) queue;
    list_add(&q->lh, (list_t *)msg);
#ifdef DEBUG_OS
    cfw_log("queue_put: %p <- %p\n", queue, msg);
#endif
}

void * queue_wait(void *queue) {
    q_t * q = (q_t*) queue;
    void * elem = (void *)list_get(&q->lh);
#ifdef DEBUG_OS
    cfw_log("queue_wait: %p -> %p\n", queue, elem);
#endif
    return elem;
}

void queue_get_message (T_QUEUE queue, T_QUEUE_MESSAGE* message, int timeout, OS_ERR_TYPE* err) {
    *message = queue_wait(queue);
}

void queue_send_message (T_QUEUE queue, T_QUEUE_MESSAGE message, OS_ERR_TYPE* err) {
    queue_put(queue, message);
}

T_QUEUE queue_create(uint32_t  max_size, OS_ERR_TYPE*err) {
    int i, found=0;
    q_t * q;
    for (i=0;i<10; i++) {
        q = &q_pool[i];
        if (q->used == 0) {
            q->used = 1;
            found = 1;
        }
    }
    if (!found) return (T_QUEUE)NULL;
    list_init(&q->lh);
    q->count = 0;
    return (T_QUEUE) q;
}

void queue_delete(T_QUEUE queue, OS_ERR_TYPE* err) {
    void * element = NULL;
    q_t * q = (q_t*) queue;
    while((element = list_get(&q->lh)) != NULL)
        list_remove(&q->lh, element);
    cfw_free(q, NULL);
}

/*************************    MUTEXES   *************************/
T_MUTEX mutex_create(OS_ERR_TYPE* err)
{
    return (T_MUTEX) NULL;
}

void mutex_delete(T_MUTEX mutex, OS_ERR_TYPE* err)
{
    return;
}

/* Stub - not implemented */
void mutex_unlock(T_MUTEX mutex, OS_ERR_TYPE* err)
{
    return;
}

/* Stub - not implemented */
OS_ERR_TYPE mutex_lock(T_MUTEX mutex, int timeout)
{
    return E_OS_OK;
}
