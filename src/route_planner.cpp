#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    // Store the nodes in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}


// Implement the CalculateHValue method to calculate the heuristic value
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Expand the current node by adding all unvisited neighbors to the open list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate current_node.neighbors
    current_node->FindNeighbors();

    // Compute attributes of each neighbor and add to open_list
    for (RouteModel::Node *neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = RoutePlanner::CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;

        open_list.push_back(neighbor);
    }

}

/* 
Sort the open list and return the next node.
*/

// Comparator function for open_list sorting
bool RoutePlanner::GreaterSumGH (const RouteModel::Node* node1, const RouteModel::Node* node2) {
    if ((node1->g_value + node1->h_value) >= (node1->g_value + node1->h_value)) {
        return 1;
    } else {
        return 0;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list with highest g+h at front, lowest g+h at back
    std::sort(open_list.begin(), open_list.end(), GreaterSumGH);

    // Return the node with the lowest g+h sum, and remove it from open_list
    RouteModel::Node* NextNodePointer = open_list.back();
    open_list.pop_back(); // Remove node with lowest GH sum
    return NextNodePointer;
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