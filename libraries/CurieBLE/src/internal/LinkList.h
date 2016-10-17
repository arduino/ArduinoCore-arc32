/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 *
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef _LINKLIST_H_
#define _LINKLIST_H_

template<typename T> struct LinkNode {
    LinkNode<T> *next;
    T value;
};

template<typename T> LinkNode<T>* link_node_create(T value)
{
    LinkNode<T>* node = (LinkNode<T>*)malloc(sizeof(LinkNode<T>));

    if (node) {
      node->value = value;
      node->next = NULL;
    }
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

