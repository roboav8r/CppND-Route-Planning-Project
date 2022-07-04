#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

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
    for (auto& neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = RoutePlanner::CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;

        open_list.push_back(neighbor);
    }

}

// Sort the open list of nodes and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list with highest g+h at front, lowest g+h at back
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node* n1, const RouteModel::Node* n2){return (n1->g_value+n1->h_value > n2->g_value+n2->h_value);});

    // Return the node with the lowest g+h sum, and remove it from open_list
    RouteModel::Node* NextNodePointer = open_list.back();
    open_list.pop_back(); // Remove node with lowest g+h sum
    return NextNodePointer;
}


// Return the final path found from A* search
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Add current (final) node to the path vector 
    path_found.push_back(*current_node);

    // Iterate through the parents of the current node until the nullptr is reached (default initial parent value)
    RouteModel::Node *path_node = current_node;
    while (path_node->parent != nullptr) {

        // Insert path_node in back of path_found; update total distance
        path_found.push_back(*path_node->parent);
        distance += path_node->distance(*(path_node->parent));

        // Increment from current path node to its parent
        path_node = path_node->parent;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end()); // Reverse path_found to place start_node at front() and end_node at back()
    return path_found;

}

/*
A* Search algorithm
*/
void RoutePlanner::AStarSearch() {
    // Initialize current node & loop control flag
    RouteModel::Node *current_node = nullptr;

    // Add starting node to vector of open nodes; mark as visited
    open_list.push_back(start_node);
    start_node->visited=true; // Added to fix infinite loop per https://knowledge.udacity.com/questions/517436

    // Main loop
    while (open_list.empty() == false) {

        // Select next node from open_list
        current_node = NextNode();

        // If the goal has been reached, save the final path and exit AStarSearch()
        if (current_node==end_node) { // Replaced ReachedGoal check with this per https://knowledge.udacity.com/questions/517436
            m_Model.path = ConstructFinalPath(current_node);
            return;

        } else { // Otherwise, check for other nodes
            AddNeighbors(current_node);
        }
    }
}
