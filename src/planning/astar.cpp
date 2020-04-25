#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <common/grid_utils.hpp>
#include <queue>
#include <iostream>
#include <functional>

typedef Point<int> cell_t;


struct Node
{
    cell_t parentCell;
    cell_t cell;
    int hCost;
    int gCost;
    int fCost;       
    
    bool operator<(const Node& rhs) const
    {
        return fCost < rhs.fCost;
    }

    bool operator>(const Node& rhs) const 
    {
        return fCost > rhs.fCost;
    }
};
 
void makePath(robot_path_t* path, Node startNode, Node goalNode, const ObstacleDistanceGrid& distances, std::vector<Node> closedQueue);
bool isGoal(const cell_t currCell, const cell_t goalCell);
bool isMember(Node currNode, std::vector<Node> closedQueue);
std::vector<Node> expandNode(Node currNode);
float getHCost(const cell_t currCell, const cell_t goalCell );
float getGCost(const Node parentNode, const cell_t kidCell);
Node getNode(cell_t cell, std::vector<Node> closedQueue);


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    robot_path_t path;
    path.utime = start.utime;

    Point<float> startCell_point;
    startCell_point.x = start.x;
    startCell_point.y = start.y;
    cell_t startCell = global_position_to_grid_cell(startCell_point, distances);
   
    Point<float> goalCell_point;
    goalCell_point.x = goal.x;
    goalCell_point.y = goal.y;
    cell_t goalCell = global_position_to_grid_cell(goalCell_point, distances);


    if (isGoal(startCell, goalCell)) {
        path.path.push_back(start);
        path.path_length = path.path.size();
        return path;
    } else {
        std::priority_queue<Node, std::vector<Node>,std::greater<Node>> openQueue;
        std::vector<Node> closedQueue;

        Node startNode;
        startNode.cell = startCell;
        startNode.gCost = 0;
        startNode.hCost = 0;
        startNode.fCost = 0;
        startNode.parentCell = startCell;
        openQueue.push(startNode);
       
       
        while (! openQueue.empty()) {
            Node currNode = openQueue.top();
            openQueue.pop();
            if(!isMember(currNode, closedQueue)) {
                std::vector<Node> kids = expandNode(currNode);
                for (auto& kid: kids) {
                    if (distances.isCellInGrid(kid.cell.x, kid.cell.y) && distances(kid.cell.x,kid.cell.y) > params.minDistanceToObstacle) {
                    // if (distances.isCellInGrid(kid.cell.x, kid.cell.y) && distances(kid.cell.x,kid.cell.y) > 0) {
                        kid.parentCell = currNode.cell;
                        kid.gCost = getGCost(currNode, kid.cell);
                        kid.hCost = getHCost(kid.cell, goalCell);
                        kid.fCost = kid.gCost + kid.hCost;
                        if (distances(kid.cell.x,kid.cell.y) < params.maxDistanceWithCost) {
                            kid.fCost += pow(params.maxDistanceWithCost - distances(kid.cell.x, kid.cell.y), params.distanceCostExponent);
                        }
                        if (isGoal(kid.cell, goalCell)) {
                            std::cout<<"FOUND GOAL, GO MAKE PATH"<<std::endl;
                            std::cout<<openQueue.size()<<std::endl;
                            path.path.push_back(start);
                            closedQueue.push_back(currNode);
                            makePath(&path, startNode, kid, distances, closedQueue);
                            return path;
                        }
                        bool member = isMember(kid, closedQueue);
                        if (!member) {

                            openQueue.push(kid);
                        }
                    }
                }
                closedQueue.push_back(currNode);
            }
        }
    } 
}



float getGCost(const Node parentNode, const cell_t kidCell) {
    int x = abs(parentNode.cell.x - kidCell.x);
    int y = abs(parentNode.cell.y - kidCell.y);
    if(x==1 && y==1){
        return parentNode.gCost + 14;
    } else {
        return parentNode.gCost + 10;
    }
}

float getHCost(const cell_t currCell, const cell_t goalCell ) {
    int x = abs(goalCell.x - currCell.x);
    int y = abs(goalCell.y - currCell.y);
    int cost;
    if(x >= y) {
        cost = 14 * y + 10 * (x - y);
    } else {
        cost = 14 * x + 10 * (y - x);
    }
    return cost;
}

std::vector<Node> expandNode(Node currNode) {
    const int xDeltas[8] = { 1, -1, 0,  0, 1, 1, -1, -1}; 
    const int yDeltas[8] = {0,  0, 1, -1, 1, -1, 1, -1 };

    std::vector<Node> kids;
    for(int n = 0; n < 8; ++n) {
        cell_t adjacentCell(currNode.cell.x + xDeltas[n], currNode.cell.y + yDeltas[n]);
        Node kid;
        kid.cell = adjacentCell;
        kids.push_back(kid);
        // if(grid.isCellInGrid(adjacentCell.x, adjacentCell.y)) { // check if cell is not obstacle + not in closed list
        //     Node kid;
        //     kid.parent_cell = currNode.cell;
        //     kid.cell = adjacentCell;
        //     kid.gCost = getGCost(currNode, kid.cell);
        //     kid.hCost = getHCost(kid.cell, goalCell);
        //     kid.fCost = kid.gCost + kid.hCost;
        //     kids.push_back(kid);
        // }
    }
    return kids;
}


bool isMember(Node currNode, std::vector<Node> closedQueue) {
    for (auto& v: closedQueue) {
        if (v.cell.x == currNode.cell.x && v.cell.y == currNode.cell.y) {
            if (v.parentCell.x == currNode.parentCell.x && v.parentCell.y == currNode.parentCell.y) {
                if (currNode.fCost < v.fCost) {
                    v = currNode;
                }
                return true;
            }
        }
    }
    return false;
}


bool isGoal(const cell_t currCell, const cell_t goalCell) {
    if (currCell.x == goalCell.x && currCell.y == goalCell.y) {
        return true;
    }
    return false;
}

Node getNode(cell_t cell, std::vector<Node> closedQueue) {
    for(auto& q : closedQueue) {
        if (cell.x == q.cell.x && cell.y == q.cell.y) {
            return q;
        }
    }
}


void makePath(robot_path_t* path, Node startNode, Node goalNode, const ObstacleDistanceGrid& distances, std::vector<Node> closedQueue ) {
    std::vector<cell_t> reverseCellList;
    Node currNode = goalNode;
    int i = 0;
    while(startNode.cell.x != currNode.cell.x || startNode.cell.y != currNode.cell.y) {
        cell_t currCell = currNode.cell;
        reverseCellList.push_back(currCell);
        currNode = getNode(currNode.parentCell, closedQueue);
        i += 1;
        if (i == 10000) {
            printf("BREAK\n");
            break;
        }
    }
    for(int j = reverseCellList.size() - 1; j >= 0; j --) {
        pose_xyt_t prevPose = path->path.back();
        pose_xyt_t currPose;
        Point<int> currCell = reverseCellList[j];
        // std::cout<<"CELL:"<<currCell.x<<","<<currCell.y<<std::endl;
        Point<float> currPoint = grid_position_to_global_position(currCell, distances);
        currPose.x = currPoint.x;
        currPose.y = currPoint.y;
        float dx = currPoint.x - prevPose.x;
        float dy = currPoint.y - prevPose.y;
        currPose.theta = atan2(dy, dx);
        path->path.push_back(currPose);
    }
    path->path_length = path->path.size();
}
