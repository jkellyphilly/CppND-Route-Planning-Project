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
    // 1. set the parent of the neighbor to the current node
    // 2. set the h value (using CalculateHValue)
    // 3. set the g value = current node's g value plus distance b/w current node
    //    and this neighbor
    // 4. mark the neighbor as having been visited
    // 5. add the neighbor to the open list of nodes
    for (auto nodeptr : current_node->neighbors) {
        (*nodeptr).parent = current_node; 
        (*nodeptr).h_value = RoutePlanner::CalculateHValue(nodeptr);
        (*nodeptr).g_value = current_node->g_value + current_node->distance(*nodeptr);
        (*nodeptr).visited = true;
        open_list.emplace_back(nodeptr);
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


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

}