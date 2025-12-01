#include "maze_solver/pathfinder.hpp"
#include <algorithm>
#include <cmath>

namespace maze_solver {

PathFinder::PathFinder(const MazeGraph& graph) : graph_(graph) {}

double PathFinder::heuristic(const Position& pos, const Position& goal) const {
    // Distância de Manhattan
    return static_cast<double>(pos.manhattan_distance(goal));
}

std::vector<Position> PathFinder::find_path(const Position& start, const Position& goal) {
    // Priority queue para nós a explorar (min-heap baseado em f_cost)
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    
    // Conjuntos para rastrear nós visitados
    std::unordered_set<Position, Position::Hash> closed_set;
    std::unordered_map<Position, double, Position::Hash> g_scores;
    std::unordered_map<Position, Position, Position::Hash> came_from;
    
    // Inicializar com o nó inicial
    open_set.emplace(start, 0.0, heuristic(start, goal), start);
    g_scores[start] = 0.0;
    
    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();
        
        // Se chegamos ao alvo, reconstruir caminho
        if (current.pos == goal) {
            return reconstruct_path(came_from, current.pos);
        }
        
        // Se já visitamos este nó, pular
        if (closed_set.count(current.pos)) {
            continue;
        }
        
        closed_set.insert(current.pos);
        
        // Explorar vizinhos
        for (const Position& neighbor : graph_.get_neighbors(current.pos)) {
            if (closed_set.count(neighbor)) {
                continue;
            }
            
            double tentative_g = current.g_cost + 1.0; // Custo de movimento = 1
            
            // Se encontramos um caminho melhor para este vizinho
            if (!g_scores.count(neighbor) || tentative_g < g_scores[neighbor]) {
                g_scores[neighbor] = tentative_g;
                came_from[neighbor] = current.pos;
                
                double h = heuristic(neighbor, goal);
                open_set.emplace(neighbor, tentative_g, h, current.pos);
            }
        }
    }
    
    // Nenhum caminho encontrado
    return std::vector<Position>();
}

std::vector<Position> PathFinder::reconstruct_path(
    const std::unordered_map<Position, Position, Position::Hash>& came_from,
    const Position& current) const {
    
    std::vector<Position> path;
    Position pos = current;
    
    // Reconstruir caminho do fim para o início
    while (came_from.count(pos)) {
        path.push_back(pos);
        pos = came_from.at(pos);
    }
    
    // Inverter para obter caminho do início ao fim
    std::reverse(path.begin(), path.end());
    
    return path;
}

std::vector<std::string> PathFinder::path_to_commands(const std::vector<Position>& path) {
    std::vector<std::string> commands;
    
    if (path.size() < 2) {
        return commands;
    }
    
    Position current = graph_.get_robot_position();
    
    for (size_t i = 0; i < path.size(); i++) {
        Position next = path[i];
        
        int dx = next.x - current.x;
        int dy = next.y - current.y;
        
        if (dx == 1 && dy == 0) {
            commands.push_back("right");
        } else if (dx == -1 && dy == 0) {
            commands.push_back("left");
        } else if (dx == 0 && dy == 1) {
            commands.push_back("down");
        } else if (dx == 0 && dy == -1) {
            commands.push_back("up");
        }
        
        current = next;
    }
    
    return commands;
}

} // namespace maze_solver