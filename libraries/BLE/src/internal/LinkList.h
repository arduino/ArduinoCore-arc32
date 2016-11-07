#ifndef _LINKLIST_H_
#define _LINKLIST_H_

template<typename T> struct LinkNode {
    LinkNode<T> *next;
    T value;
};

template<typename T> LinkNode<T>* link_node_create(T value)
{
    LinkNode<T>* node = (LinkNode<T>*)malloc(sizeof(LinkNode<T>));
    node->value = value;
    node->next = NULL;
    return node;
}

template<typename T> void link_node_insert_last(LinkNode<T> *root, LinkNode<T> *node)
{
    while(root->next != 0)
    {
        root = root->next;
    }
    root->next = node;
}

template<typename T> void link_node_remove_last(LinkNode<T> *root)
{
    LinkNode<T> *temp1, *temp2;
    if (root->next != NULL)
    {
        temp1 = root->next;
        while(temp1->next != NULL)
        {
            temp2 = temp1;
            temp1 = temp1->next;
        }
        
        free(temp1);
        temp2->next = NULL;
    }
}

template<typename T> void link_node_remove_first(LinkNode<T> *root)
{
    LinkNode<T> *temp1;
    if (root->next != NULL)
    {
        temp1 = root->next;
        root->next = temp1->next;
        free(temp1);
    }
}

template<typename T> LinkNode<T> * link_node_get_first(LinkNode<T> *root)
{
    return root->next;
}

template<typename T> void link_node_insert_first(LinkNode<T> *root, LinkNode<T> *node)
{
    LinkNode<T>* temp = root->next;
    root->next = node;
    node->next = temp;
}

template<typename T> int link_list_size(const LinkNode<T> *root)
{
    int counter = 0;
    while(root->next != 0)
    {
        root = root->next;
        counter++;
    }
    return counter;
}

#endif

