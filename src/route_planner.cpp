#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Set our start node and end node to the closest Nodes available
    // based on the start/end X/Y coordinates.
    // Since start_node and end_node are pointers, need to pass them the
    // address of the closet node (using '&') that is returned
    // from FindClosestNode method.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// HValue is returned by using the distance method of the node 'node'. 
// The argument for this method is a node to compare against for total
// L2 distance (i.e. H-value), so use the end node. Dereference using '*'. 
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // first, populate the current node's neighbors vector
    current_node->FindNeighbors();

    // next, for each node in the current node's neighbors:
    // 1) check to see if the neighbor has already been visited. only proceed if not
    // 2) set the parent of the neighbor to the current node
    // 3) set the h value (using CalculateHValue)
    // 4) set the g value = current node's g value plus distance b/w current node
    //    and this neighbor
    // 5) mark the neighbor as having been visited
    // 6) add the neighbor to the open list of nodes
    for (auto nodeptr : current_node->neighbors) {
        if (!nodeptr->visited) {
            nodeptr->parent = current_node; 
            nodeptr->h_value = RoutePlanner::CalculateHValue(nodeptr);
            nodeptr->g_value = current_node->g_value + current_node->distance(*nodeptr);
            nodeptr->visited = true;
            open_list.emplace_back(nodeptr);
        }
    }
}

// Return true for an f-value (i.e. g + h) for n1 > n2
bool CompareNodes(RouteModel::Node* n1, RouteModel::Node* n2) {
    return (n1->g_value + n1->h_value) > (n2->g_value + n2->h_value);
}

// Sort a list of nodes in descending order comparing f-values.
// This means the lowest cost (i.e. optimal node to visit) after
// sorting would be the last one. 
void SortOpenList(std::vector<RouteModel::Node*> *open) {
    sort(open->begin(), open->end(), CompareNodes);
}

RouteModel::Node *RoutePlanner::NextNode() {
    // sort the open list
    SortOpenList(&open_list);

    // get the optimal next node to visit (i.e. lowest f-value),
    // remove it from the open list, and return a ptr to it
    RouteModel::Node* current = open_list.back();
    open_list.pop_back();
    return current;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // continue to move "up the chain" of nodes
    // until we get to the starting node (i.e. the parent
    // of the current node is a nullptr)
    while (current_node->parent != nullptr)
    {
        // add the current node to the path_found vector
        path_found.emplace_back(*current_node);

        // add the distance from the current node to the parent
        // to the existing distance variable
        distance += current_node->distance(*(current_node->parent));

        // update the current node to be its parent
        current_node = current_node->parent;
    }
    
    // now we just want to add the starting node to the list
    path_found.emplace_back(*current_node);

    // the list is in end-to-start, so reverse it
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // first, set current_node to the start_node
    // and fill out its properties
    current_node = start_node;
    current_node->g_value = 0;
    current_node->h_value = RoutePlanner::CalculateHValue(current_node);
    current_node->parent = nullptr;

    // put current node into open list
    open_list.emplace_back(current_node);

    // while the open list isn't empty, 
    // 1) sort the open list and return the next node with NextNode()
    // 2) check to see if the optimal node is the goal. if it is, set the model's
    //    path with the ConstructFinalPath method and break out
    // 3) if we aren't at the goal, set the current node's visited attribute to true
    //    and populate the open_list vector with its neighbors through AddNeighbors method
    while(!open_list.empty()) 
    {
        current_node = RoutePlanner::NextNode();

        // goal check
        if (current_node == end_node) {
            m_Model.path = RoutePlanner::ConstructFinalPath(current_node);
            break;
        }

        current_node->visited = true;
        RoutePlanner::AddNeighbors(current_node);
    }
}