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

// Classe para mapear o labirinto usando Algoritmo de Trémaux
class Mapper {
public:
    Mapper(int estimated_width, int estimated_height);
    
    // Atualizar mapa com dados do sensor
    void update_from_sensor(const Position& robot_pos, const SensorData& sensor);
    
    // Obter o grafo mapeado
    MazeGraph& get_graph() { return graph_; }
    const MazeGraph& get_graph() const { return graph_; }
    
    // Algoritmo de Trémaux: decidir próximo movimento
    std::string tremaux_next_move(const Position& current_pos, const SensorData& sensor);
    
    // Verificar se a exploração está completa
    bool is_exploration_complete() const;
    
    // Marcar célula como visitada
    void mark_visited(const Position& pos);
    
    // Obter contador de visitas de uma posição
    int get_visit_count(const Position& pos) const;
    
private:
    MazeGraph graph_;
    std::unordered_map<Position, int, Position::Hash> visit_count_;  // Contador de visitas (Trémaux)
    std::vector<Position> path_history_;  // Histórico do caminho
    
    // Converter string do sensor em CellType
    CellType sensor_string_to_cell_type(const std::string& sensor_value) const;
    
    // Verificar se uma direção é válida (não parede, não muito visitada)
    bool is_direction_valid(const std::string& direction, const SensorData& sensor) const;
};

} // namespace maze_solver

#endif // MAZE_SOLVER_MAPPER_HPP