#include "linked_list.h"

void linked_list_init(linked_list_t* list)
{
    list->head = NULL; // Initializes list variables to default values
    list->length = 0;
}

void linked_list_add_node_end(linked_list_t* list, linked_list_node_t* node)
{
    linked_list_node_t* val = list->head; // Get first node
    while (val->next != NULL) // Loop until the last one
    {
        val = val->next;
    }
    val->next = node; // Add node
    list->length++; // Increment length
}

void linked_list_add_node(linked_list_t* list, linked_list_node_t* node, uint16_t index)
{
    if (list->length <= index) // If specified index is outside of the list length
    {
        linked_list_add_node_end(list, node); // Add node to the end
    }
    else
    {
        linked_list_node_t* val = list->head; // Get first node
        for (int i = 0; i < index; i++) // Loop until the specified position
        {
            val = val->next;
        }
        val->next = node; // Add node
        list->length++; // Increment length
    }
}

linked_list_node_t* linked_list_remove_node(linked_list_t* list, uint16_t index)
{
    linked_list_node_t* node;
    if (list->length < index && index > 0) // If non-zero index inside list
    {
        linked_list_node_t* val = list->head; // Get first node
        for (int i = 0; i < index - 1; i++) // Loop until one node before the one to be deleted
        {
            val = val->next;
        }
        node = val->next;
        val->next = val->next->next; // Remove node from the linked list
        list->length--; // Decrement length
    }
    else if (list->length < index && index == 0) // If index is zero
    {
        node = list->head;
        list->head = list->head->next; // Update first node
        list->length--; // Decrement length
    }
    else // If outside of list
    {
        linked_list_node_t* val = list->head; // Get first node
        while (val->next != NULL) // Loop until the end of the list
        {
            val = val->next;
        }
        node = val->next;
        val->next = NULL; // Remove last node
        list->length--; // Decrement length
    }
    return node;
}

linked_list_node_t* linked_list_remove_last(linked_list_t* list)
{
    linked_list_node_t* val = list->head; // Get first node
    while (val->next != NULL) // Loop until the end of the list
    {
        val = val->next;
    }
    linked_list_node_t* node = val->next;
    val->next = NULL; // Remove last node
    list->length--; // Decrement length
    return node;
}

linked_list_node_t* linked_list_remove_first(linked_list_t* list)
{
    linked_list_node_t* node = list->head;
    list->head = list->head->next;
    list->length--;
    return node;
}


