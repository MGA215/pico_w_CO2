/**
 * @file linked_list_node.h
 * @author Martin Garncarz (246815@vutbr.cz)
 * @brief Header file of the linked list node structure
 * @version 0.1
 * @date 2024-06-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/**
 * @brief structure of a node in a linked list
 * 
 */
typedef struct linked_list_node {
    void* value; // Pointer to the value of the node
    linked_list_node_t* next; // Pointer to the next node
} linked_list_node_t;