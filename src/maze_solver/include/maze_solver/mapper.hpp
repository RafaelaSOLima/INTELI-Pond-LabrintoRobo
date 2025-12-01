#ifndef MAZE_SOLVER_MAPPER_HPP
#define MAZE_SOLVER_MAPPER_HPP

#include "maze_solver/graph.hpp"
#include <unordered_set>
#include <queue>

namespace maze_solver {

// Estrutura para dados do sensor
struct SensorData {
    std::string up;
    std::string down;
    std::string left;
    std::string right;
    std::string up_left;
    std::string up_right;
    std::string down_left;
    std::string down_right;
};

// Classe para mapear o labirinto usando DFS ou exploração
class Mapper {
public:
    Mapper(int estimated_width, int estimated_height);
    
    // Atualizar mapa com dados do sensor
    void update_from_sensor(const Position& robot_pos, const SensorData& sensor);
    
    // Obter o grafo mapeado
    MazeGraph& get_graph() { return graph_; }
    const MazeGraph& get_graph() const { return graph_; }
    
    // Encontrar próxima posição não explorada (fronteira)
    Position find_next_exploration_target(const Position& current);
    
    // Verificar se a exploração está completa
    bool is_exploration_complete() const;
    
    // Marcar célula como visitada
    void mark_visited(const Position& pos);
    
private:
    MazeGraph graph_;
    std::unordered_set<Position, Position::Hash> visited_;
    std::unordered_set<Position, Position::Hash> frontier_;
    
    // Converter string do sensor em CellType
    CellType sensor_string_to_cell_type(const std::string& sensor_value) const;
    
    // Adicionar posições de fronteira (células desconhecidas adjacentes a conhecidas)
    void update_frontier(const Position& pos);
};

} // namespace maze_solver

#endif // MAZE_SOLVER_MAPPER_HPP