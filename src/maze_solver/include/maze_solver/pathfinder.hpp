#ifndef MAZE_SOLVER_PATHFINDER_HPP
#define MAZE_SOLVER_PATHFINDER_HPP

#include "maze_solver/graph.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <string>

namespace maze_solver {

// Nó para o algoritmo A*
struct AStarNode {
    Position pos;
    double g_cost;  // Custo do início até este nó
    double h_cost;  // Heurística (custo estimado até o alvo)
    double f_cost;  // g_cost + h_cost
    Position parent;
    
    AStarNode(Position p, double g, double h, Position par) 
        : pos(p), g_cost(g), h_cost(h), f_cost(g + h), parent(par) {}
    
    // Para priority_queue (menor f_cost tem prioridade)
    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
};

// Classe para encontrar caminhos no labirinto
class PathFinder {
public:
    PathFinder(const MazeGraph& graph);
    
    // Algoritmo A* para encontrar o caminho ótimo
    std::vector<Position> find_path(const Position& start, const Position& goal);
    
    // Converte caminho em comandos de movimento
    std::vector<std::string> path_to_commands(const std::vector<Position>& path);
    
private:
    const MazeGraph& graph_;
    
    // Heurística para A* (distância de Manhattan)
    double heuristic(const Position& pos, const Position& goal) const;
    
    // Reconstruir caminho a partir dos nós visitados
    std::vector<Position> reconstruct_path(
        const std::unordered_map<Position, Position, Position::Hash>& came_from,
        const Position& current) const;
};

} // namespace maze_solver

#endif // MAZE_SOLVER_PATHFINDER_HPP