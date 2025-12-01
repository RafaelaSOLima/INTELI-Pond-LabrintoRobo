#ifndef MAZE_SOLVER_GRAPH_HPP
#define MAZE_SOLVER_GRAPH_HPP

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <utility>

namespace maze_solver {

// Estrutura para representar uma posição no labirinto
struct Position {
    int x;
    int y;

    Position(int x = 0, int y = 0) : x(x), y(y) {}

    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }

    // Hash para usar Position em unordered_map
    struct Hash {
        size_t operator()(const Position& pos) const {
            return std::hash<int>()(pos.x) ^ (std::hash<int>()(pos.y) << 1);
        }
    };

    // Distância de Manhattan
    int manhattan_distance(const Position& other) const {
        return abs(x - other.x) + abs(y - other.y);
    }
};

// Tipos de células do labirinto
enum class CellType {
    EMPTY,      // Branco - passável
    WALL,       // Preto - bloqueado
    ROBOT,      // Azul - robô
    TARGET,     // Vermelho - alvo
    UNKNOWN     // Desconhecido (para mapeamento)
};

// Classe para representar o grafo do labirinto
class MazeGraph {
public:
    MazeGraph(int width, int height);
    
    // Configuração do mapa
    void set_cell(int x, int y, CellType type);
    CellType get_cell(int x, int y) const;
    
    // Posições especiais
    void set_robot_position(const Position& pos);
    void set_target_position(const Position& pos);
    Position get_robot_position() const { return robot_pos_; }
    Position get_target_position() const { return target_pos_; }
    
    // Verificações
    bool is_valid_position(const Position& pos) const;
    bool is_walkable(const Position& pos) const;
    
    // Vizinhos (apenas 4 direções: cima, baixo, esquerda, direita)
    std::vector<Position> get_neighbors(const Position& pos) const;
    
    // Dimensões
    int get_width() const { return width_; }
    int get_height() const { return height_; }
    
    // Carregar mapa de dados flat (do serviço /get_map)
    void load_from_flat_data(const std::vector<std::string>& data, int width, int height);
    
private:
    int width_;
    int height_;
    std::vector<std::vector<CellType>> grid_;
    Position robot_pos_;
    Position target_pos_;
};

} // namespace maze_solver

#endif // MAZE_SOLVER_GRAPH_HPP