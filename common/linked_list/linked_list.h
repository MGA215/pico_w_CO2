/**
 * @file linked_list.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Library for linked list
 * @version 0.1
 * @date 2024-06-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "linked_list_node.h"

/**
 * @brief linked list structure
 * 
 */
typedef struct linked_list
{
    linked_list_node_t* head; // First node
    uint16_t length; // Length of the linked list

} linked_list_t;

/**
 * @brief Initializes linked list
 * 
 * @param list Linked list to be initialized
 */
void linked_list_init(linked_list_t* list);

/**
 * @brief Adds node to the end of the linked list
 * 
 * @param list Linked list for the node to be added to
 * @param node Node to be added
 */
void linked_list_add_node_end(linked_list_t* list, linked_list_node_t* node);

/**
 * @brief Adds node to a location in the linked list specified by index
 * 
 * @param list Linked list for the node to be added to
 * @param node Node to be added
 * @param index Position of the node in the linked list
 */
void linked_list_add_node(linked_list_t* list, linked_list_node_t* node, uint16_t index);

/**
 * @brief Removes node from the linked list on the position specified by index
 * 
 * @param list Linked list for the node to be removed from
 * @param index Position of the node in the linked list
 * @return linked_list_node_t* Removed node
 */
linked_list_node_t* linked_list_remove_node(linked_list_t* list, uint16_t index);

/**
 * @brief Removes node from the end of the linked list
 * 
 * @param list Linked list for the node to be removed from
 * @return linked_list_node_t* Removed node
 */
linked_list_node_t* linked_list_remove_last(linked_list_t* list);

/**
 * @brief Removes node from the beginning of the linked list
 * 
 * @param list Linked list for the node to be removed from
 * @return linked_list_node_t* Removed node
 */
linked_list_node_t* linked_list_remove_first(linked_list_t* list);
