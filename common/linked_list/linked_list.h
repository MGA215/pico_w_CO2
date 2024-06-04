#include "linked_list_node.h"


typedef struct linked_list
{
    linked_list_node_t* head;
    uint16_t length;

} linked_list_t;

void linked_list_init(linked_list_t* list);

void linked_list_add_node_end(linked_list_t* list, linked_list_node_t* node);

void linked_list_add_node(linked_list_t* list, linked_list_node_t* node, uint16_t index);

void linked_list_remove_node(linked_list_t* list, uint16_t index);

void linked_list_remove_last(linked_list_t* list);

void linked_list_remove_first(linked_list_t* list);
