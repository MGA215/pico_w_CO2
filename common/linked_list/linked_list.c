#include "linked_list.h"

void linked_list_init(linked_list_t* list)
{
    list->head = NULL;
    list->length = 0;
}

void linked_list_add_node_end(linked_list_t* list, linked_list_node_t* node)
{
    linked_list_node_t* val = list->head;
    while (val->next != NULL)
    {
        val = val->next;
    }
    val->next = node;
    list->length++;
}

void linked_list_add_node(linked_list_t* list, linked_list_node_t* node, uint16_t index)
{
    if (list->length <= index)
    {
        linked_list_add_node_end(list, node);
    }
    else
    {
        linked_list_node_t* val = list->head;
        for (int i = 0; i < index; i++)
        {
            val = val->next;
        }
        val->next = node;
        list->length++;
    }
}

void linked_list_remove_node(linked_list_t* list, uint16_t index)
{
    if (list->length < index && index > 0)
    {
        linked_list_node_t* val = list->head;
        for (int i = 0; i < index - 1; i++)
        {
            val = val->next;
        }
        val->next = val->next->next;
        list->length--;
    }
    else if (list->length < index && index == 0)
    {
        list->head = list->head->next;
        list->length--;
    }
}

void linked_list_remove_last(linked_list_t* list)
{
    linked_list_node_t* val = list->head;
    while (val->next != NULL)
    {
    val = val->next;
    }
    val->next = val->next->next;
    list->length--;
}

void linked_list_remove_first(linked_list_t* list)
{
    list->head = list->head->next;
    list->length--;
}


