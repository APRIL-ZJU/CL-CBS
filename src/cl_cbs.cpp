/**
 * @file cl_cbs.cpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief The implement of CL-CBS
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "cl_cbs.hpp"

#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>

#include "environment.hpp"
#include "timer.hpp"

// =====================================================================
// 【追記箇所①】 cl_cbs.cpp の一番上（#include が並んでいる場所）に追加
// =====================================================================
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
// 追記箇所①

using libMultiRobotPlanning::CL_CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace libMultiRobotPlanning;

// calculate agent collision more precisely BUT need LONGER time
// #define PRCISE_COLLISION

struct Location {
  Location(double x, double y) : x(x), y(y) {}
  double x;
  double y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct State {
  State(double x, double y, double yaw, int time = 0)
      : time(time), x(x), y(y), yaw(yaw) {
    rot.resize(2, 2);
    rot(0, 0) = cos(-this->yaw);
    rot(0, 1) = -sin(-this->yaw);
    rot(1, 0) = sin(-this->yaw);
    rot(1, 1) = cos(-this->yaw);
#ifdef PRCISE_COLLISION
    corner1 = Point(
        this->x -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LB) - this->yaw),
        this->y -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LB) - this->yaw));
    corner2 = Point(
        this->x -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LB) + this->yaw),
        this->y +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LB) + this->yaw));
    corner3 = Point(
        this->x +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LF) - this->yaw),
        this->y +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LF) - this->yaw));
    corner4 = Point(
        this->x +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LF) + this->yaw),
        this->y -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LF) + this->yaw));
#endif
  }

  State() = default;

  bool operator==(const State& s) const {
    return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
  }

  bool agentCollision(const State& other) const {
#ifndef PRCISE_COLLISION
    if (pow(this->x - other.x, 2) + pow(this->y - other.y, 2) <
        pow(2 * Constants::LF, 2) + pow(Constants::carWidth, 2))
      return true;
    return false;
#else
    std::vector<Segment> rectangle1{Segment(this->corner1, this->corner2),
                                    Segment(this->corner2, this->corner3),
                                    Segment(this->corner3, this->corner4),
                                    Segment(this->corner4, this->corner1)};
    std::vector<Segment> rectangle2{Segment(other.corner1, other.corner2),
                                    Segment(other.corner2, other.corner3),
                                    Segment(other.corner3, other.corner4),
                                    Segment(other.corner4, other.corner1)};
    for (auto seg1 = rectangle1.begin(); seg1 != rectangle1.end(); seg1++)
      for (auto seg2 = rectangle2.begin(); seg2 != rectangle2.end(); seg2++) {
        if (boost::geometry::intersects(*seg1, *seg2)) return true;
      }
    return false;
#endif
  }

  bool obsCollision(const Location& obstacle) const {
    boost::numeric::ublas::matrix<double> obs(1, 2);
    obs(0, 0) = obstacle.x - this->x;
    obs(0, 1) = obstacle.y - this->y;

    auto rotated_obs = boost::numeric::ublas::prod(obs, rot);
    if (rotated_obs(0, 0) > -Constants::LB - Constants::obsRadius &&
        rotated_obs(0, 0) < Constants::LF + Constants::obsRadius &&
        rotated_obs(0, 1) > -Constants::carWidth / 2.0 - Constants::obsRadius &&
        rotated_obs(0, 1) < Constants::carWidth / 2.0 + Constants::obsRadius)
      return true;
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")@" << s.time;
  }

  int time;
  double x;
  double y;
  double yaw;

 private:
  boost::numeric::ublas::matrix<double> rot;
  Point corner1, corner2, corner3, corner4;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.yaw);
    return seed;
  }
};
}  // namespace std

using Action = int;  // int<7 int ==6 wait

struct Conflict {
  int time;
  size_t agent1;
  size_t agent2;

  State s1;
  State s2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    os << c.time << ": Collision [ " << c.agent1 << c.s1 << " , " << c.agent2
       << c.s2 << " ]";
    return os;
  }
};

struct Constraint {
  Constraint(int time, State s, size_t agentid)
      : time(time), s(s), agentid(agentid) {}
  Constraint() = default;
  int time;
  State s;
  size_t agentid;

  bool operator<(const Constraint& other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) <
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                    other.agentid);
  }

  bool operator==(const Constraint& other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) ==
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw,
                    other.agentid);
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraint& c) {
    return os << "Constraint[" << c.time << "," << c.s << "from " << c.agentid
              << "]";
  }

  bool satisfyConstraint(const State& state) const {
    if (state.time < this->time ||
        state.time > this->time + Constants::constraintWaitTime)
      return true;
    return !this->s.agentCollision(state);
  }
};

namespace std {
template <>
struct hash<Constraint> {
  size_t operator()(const Constraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.s.x);
    boost::hash_combine(seed, s.s.y);
    boost::hash_combine(seed, s.s.yaw);
    boost::hash_combine(seed, s.agentid);
    return seed;
  }
};
}  // namespace std

// FIXME: modidy data struct, it's not the best option
struct Constraints {
  std::unordered_set<Constraint> constraints;

  void add(const Constraints& other) {
    constraints.insert(other.constraints.begin(), other.constraints.end());
  }

  bool overlap(const Constraints& other) {
    for (const auto& c : constraints) {
      if (other.constraints.count(c) > 0) return true;
    }
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& cs) {
    for (const auto& c : cs.constraints) {
      os << c << std::endl;
    }
    return os;
  }
};

void readAgentConfig() {
  YAML::Node car_config;
  std::string test(__FILE__);
  boost::replace_all(test, "cl_cbs.cpp", "config.yaml");
  try {
    car_config = YAML::LoadFile(test.c_str());
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[33mWARNING: Failed to load agent config file: "
              << test << "\033[0m , Using default params. \n";
  }
  // int car_r = car_config["r"].as<int>();
  Constants::r = car_config["r"].as<double>();
  Constants::deltat = car_config["deltat"].as<double>();
  Constants::penaltyTurning = car_config["penaltyTurning"].as<double>();
  Constants::penaltyReversing = car_config["penaltyReversing"].as<double>();
  Constants::penaltyCOD = car_config["penaltyCOD"].as<double>();
  // map resolution
  Constants::mapResolution = car_config["mapResolution"].as<double>();
  // change to set calcIndex resolution
  Constants::xyResolution = Constants::r * Constants::deltat;
  Constants::yawResolution = Constants::deltat;

  Constants::carWidth = car_config["carWidth"].as<double>();
  Constants::LF = car_config["LF"].as<double>();
  Constants::LB = car_config["LB"].as<double>();
  // obstacle default radius
  Constants::obsRadius = car_config["obsRadius"].as<double>();
  // least time to wait for constraint
  Constants::constraintWaitTime = car_config["constraintWaitTime"].as<double>();

  Constants::dx = {Constants::r * Constants::deltat,
                   Constants::r * sin(Constants::deltat),
                   Constants::r * sin(Constants::deltat),
                   -Constants::r * Constants::deltat,
                   -Constants::r * sin(Constants::deltat),
                   -Constants::r * sin(Constants::deltat)};
  Constants::dy = {0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat)),
                   0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat))};
  Constants::dyaw = {0, Constants::deltat,  -Constants::deltat,
                     0, -Constants::deltat, Constants::deltat};
}

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  int batchSize;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")(
      "batchsize,b", po::value<int>(&batchSize)->default_value(10),
      "batch size for iter");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  readAgentConfig();

  YAML::Node map_config;
  try {
    map_config = YAML::LoadFile(inputFile);
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[31mERROR: Failed to load map file: " << inputFile
              << "\033[0m \n";
    return 0;
  }
  const auto& dim = map_config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  std::unordered_set<Location> obstacles;
  std::multimap<int, State> dynamic_obstacles;
  std::vector<State> goals;
  std::vector<State> startStates;
  for (const auto& node : map_config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<double>(), node[1].as<double>()));
  }
  for (const auto& node : map_config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(start[0].as<double>(), start[1].as<double>(),
                                   start[2].as<double>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(State(goal[0].as<double>(), goal[1].as<double>(),
                             goal[2].as<double>()));
  }

  std::cout << "Calculating Solution...\n";
  double timer = 0;
  bool success = false;
  std::vector<PlanResult<State, Action, double>> solution;
  for (size_t iter = 0; iter < (double)goals.size() / batchSize; iter++) {
    size_t first = iter * batchSize;
    size_t last = first + batchSize;
    if (last >= goals.size()) last = goals.size();
    std::vector<State> m_goals(goals.begin() + first, goals.begin() + last);
    std::vector<State> m_starts(startStates.begin() + first,
                                startStates.begin() + last);

    Environment<Location, State, Action, double, Conflict, Constraint,
                Constraints>
        mapf(dimx, dimy, obstacles, dynamic_obstacles, m_goals);
    if (!mapf.startAndGoalValid(m_starts, iter, batchSize)) {
      success = false;
      break;
    }
    for (auto goal = goals.begin() + last; goal != goals.end(); goal++) {
      dynamic_obstacles.insert(
          std::pair<int, State>(-1, State(goal->x, goal->y, goal->yaw)));
    }
    CL_CBS<State, Action, double, Conflict, Constraints,
           Environment<Location, State, Action, double, Conflict, Constraint,
                       Constraints>>
        cbsHybrid(mapf);
    std::vector<PlanResult<State, Action, double>> m_solution;
    Timer iterTimer;
    success = cbsHybrid.search(m_starts, m_solution);
    iterTimer.stop();

    if (!success) {
      std::cout << "\033[1m\033[31m No." << iter
                << "iter fail to find a solution \033[0m\n";
      break;
    } else {
      solution.insert(solution.end(), m_solution.begin(), m_solution.end());
      for (size_t a = 0; a < m_solution.size(); ++a) {
        for (const auto& state : m_solution[a].states)
          dynamic_obstacles.insert(std::pair<int, State>(
              state.first.time,
              State(state.first.x, state.first.y, state.first.yaw)));
        State lastState = m_solution[a].states.back().first;
        dynamic_obstacles.insert(std::pair<int, State>(
            -lastState.time, State(lastState.x, lastState.y, lastState.yaw)));
      }
      timer += iterTimer.elapsedSeconds();
      std::cout << "Complete " << iter
                << " iter. Runtime:" << iterTimer.elapsedSeconds()
                << " Expand high-level nodes:" << mapf.highLevelExpanded()
                << " Average Low-level-search time:"
                << iterTimer.elapsedSeconds() / mapf.highLevelExpanded() /
                       m_goals.size()
                << std::endl;
    }
    dynamic_obstacles.erase(-1);
  }

  std::ofstream out;
  out = std::ofstream(outputFile);

  if (success) {
    std::cout << "\033[1m\033[32m Successfully find solution! \033[0m\n";

    double makespan = 0, flowtime = 0, cost = 0;
    for (const auto& s : solution) cost += s.cost;

    for (size_t a = 0; a < solution.size(); ++a) {
      // calculate makespan
      double current_makespan = 0;
      for (size_t i = 0; i < solution[a].actions.size(); ++i) {
        // some action cost have penalty coefficient

        if (solution[a].actions[i].second < Constants::dx[0])
          current_makespan += solution[a].actions[i].second;
        else if (solution[a].actions[i].first % 3 == 0)
          current_makespan += Constants::dx[0];
        else
          current_makespan += Constants::r * Constants::deltat;
      }
      flowtime += current_makespan;
      if (current_makespan > makespan) makespan = current_makespan;
    }
    std::cout << " Runtime: " << timer << std::endl
              << " Makespan:" << makespan << std::endl
              << " Flowtime:" << flowtime << std::endl
              << " cost:" << cost << std::endl;
    // output to file
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  flowtime: " << flowtime << std::endl;
    out << "  runtime: " << timer << std::endl;

    // =====================================================================
// 【追記箇所②】 cl_cbs.cpp の下の方、「out << "schedule:" << std::endl;」の直前に追加
// =====================================================================

// 1. Boost.Geometry の型を定義（扱いやすいように名前をつけます）
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// 2Dの点と箱（AABB）を定義
typedef bg::model::point<double, 2, bg::cs::cartesian> Point2D;
typedef bg::model::box<Point2D> Box2D;
// R-treeに保存するデータの形：<箱, エージェントID, 時間t, 中心X, 中心Y>
typedef std::tuple<Box2D, int, int, double, double> RTreeValue;

// R-treeの本体を作成（検索を高速化するアルゴリズムとして quadratic を指定）
bgi::rtree<RTreeValue, bgi::quadratic<16>> rtree;

std::cout << "\n=== [Step 1] 経路データからR-treeを構築しています ===" << std::endl;
// 2. すべてのエージェントの経路から、R-treeを構築する
for (size_t agent_id = 0; agent_id < solution.size(); ++agent_id) {
    for (const auto& state_pair : solution[agent_id].states) {
        State s = state_pair.first;

        // 車のAABB（真っ直ぐな箱）を作る
        // 車の中心から前後左右に余裕を持たせた2.0mの箱を作ります
        double margin = 2.0; 
        Point2D min_p(s.x - margin, s.y - margin);
        Point2D max_p(s.x + margin, s.y + margin);
        Box2D car_aabb(min_p, max_p);

        // 箱のデータと、その箱が「誰の、いつの、どこのデータか」をセットにしてR-treeに登録
        RTreeValue value = std::make_tuple(car_aabb, agent_id, s.time, s.x, s.y);
        rtree.insert(value);
    }
}
std::cout << "R-treeの構築が完了しました！" << std::endl;


// // 3. 【仮の歩行者を設定】 歩行者に「大きさ（半径）」を持たせる
// // ※実際はLiDAR等からこの情報が入ってきます。テスト環境に合わせてX,Yを変更してください。
// double ped_x = 11.0;     // 歩行者のX座標
// double ped_y = 22.5;     // 歩行者のY座標
// int ped_time = 15;       // 歩行者がそこにいる時間 t
// double ped_radius = 0.5; // ★歩行者の大きさ（半径 0.5m）

// std::cout << "\n=== [Step 2] 歩行者の影響範囲をR-treeで検索します ===" << std::endl;
// // 歩行者をすっぽり覆う「検索用の箱」を作る
// Point2D ped_min_p(ped_x - ped_radius, ped_y - ped_radius);
// Point2D ped_max_p(ped_x + ped_radius, ped_y + ped_radius);
// Box2D query_box(ped_min_p, ped_max_p);

// // 検索結果を入れるための空のリスト
// std::vector<RTreeValue> query_results;

// // ★爆速検索！「この歩行者の箱(query_box)と重なっている車のデータを全部出して！」
// rtree.query(bgi::intersects(query_box), std::back_inserter(query_results));
// std::cout << "空間的に交差している（怪しい）データを " << query_results.size() << " 件抽出しました。" << std::endl;


// std::cout << "\n=== [Step 3] 時間と精密距離のチェック（ナローフェーズ） ===" << std::endl;
// bool collision_detected = false;

// // 4. R-treeが拾ってきた「怪しいリスト」だけを精密検査する
// for (const auto& result : query_results) {
//     int   agent_id  = std::get<1>(result);
//     int   state_t   = std::get<2>(result);
//     double car_x    = std::get<3>(result);
//     double car_y    = std::get<4>(result);

//     // 【判定1】まずは「時間」が一致しているかチェック！
//     if (state_t == ped_time) {
        
//         // 【判定2】時間が同じなら、歩行者と車の「実際の距離」を計算
//         double distance = sqrt(pow(car_x - ped_x, 2) + pow(car_y - ped_y, 2));

//         // 車と歩行者の距離が「車のマージン + 歩行者の半径」より近ければ衝突とみなす
//         double safe_distance = 1.0 + ped_radius; // 例: 車のマージン1.0m + 歩行者半径0.5m

//         if (distance < safe_distance) {
//             std::cout << "🚨 【衝突検知】エージェント " << agent_id 
//                       << " が時間 t=" << state_t 
//                       << " に歩行者と衝突します！ (距離: " << distance << "m)" << std::endl;
//             collision_detected = true;
            
//             // ★将来的に、ここで Online Path Repair（再計算）の関数を呼びます
//         }
//     }
// }

// if (!collision_detected) {
//     std::cout << "✅ 指定した時間・場所に衝突するエージェントはいませんでした。安全です。" << std::endl;
// }
// std::cout << "========================================================\n" << std::endl;
// //追加箇所②


// =====================================================================
// 【動く歩行者（線形）の設定】追加箇所②から歩行者が動くように変更
// =====================================================================
#include <set> // ★重複を防ぐために追加

// Boost.Geometryで「線分（Segment）」を扱うための型を定義
typedef bg::model::segment<Point2D> Segment2D;

// 歩行者の軌跡（線分）のリスト
// <始点X, 始点Y, 終点X, 終点Y, 始点時間t> で、t から t+1 への1コマの動きを「線」で表現します
std::vector<std::tuple<double, double, double, double, int>> ped_segments = {
    // t=13(10.0, 10.0) から t=14(10.5, 10.5) へ斜めに歩く線
    std::make_tuple(10.0, 22.5, 10.5, 22.5, 13), 
    // t=14(10.5, 10.5) から t=15(11.0, 11.0) へ斜めに歩く線
    std::make_tuple(10.5, 22.5, 11.0, 22.5, 14),
    // t=15(11.0, 11.0) から t=16(11.5, 11.0) へ真横に歩く線
    std::make_tuple(11.0, 22.5, 11.5, 22.5, 15),

    std::make_tuple(11.5, 22.5, 12.0, 22.5, 16),

    std::make_tuple(12.0, 22.5, 12.5, 22.5, 17)
};

double ped_radius = 0.5; // 歩行者の大きさ（マージン）

std::cout << "\n=== [Step 2 & 3] 歩行者の線形軌跡と衝突判定 ===" << std::endl;
bool collision_detected = false;

// ★重複出力防止用のセット（エージェントIDと時間のペアを記録）
std::set<std::pair<int, int>> reported_collisions;

// 4. 歩行者の「1コマ分の動く線（セグメント）」ごとにR-treeを検索する
for (const auto& ped_seg : ped_segments) {
    double start_x = std::get<0>(ped_seg);
    double start_y = std::get<1>(ped_seg);
    double end_x   = std::get<2>(ped_seg);
    double end_y   = std::get<3>(ped_seg);
    int    state_t = std::get<4>(ped_seg);

    // 【検索用の箱（AABB）を作る】
    // 歩行者が動く「線」全体をすっぽり覆う、マージン込みの長方形を作ります
    double min_x = std::min(start_x, end_x) - ped_radius;
    double max_x = std::max(start_x, end_x) + ped_radius;
    double min_y = std::min(start_y, end_y) - ped_radius;
    double max_y = std::max(start_y, end_y) + ped_radius;
    Box2D query_box(Point2D(min_x, min_y), Point2D(max_x, max_y));

    // 検索結果を入れるための空のリスト
    std::vector<RTreeValue> query_results;

    // ★爆速検索！「この歩行者の動く範囲(query_box)と重なっている車のデータを全部出して！」
    rtree.query(bgi::intersects(query_box), std::back_inserter(query_results));

    // 5. R-treeが拾ってきた「怪しいリスト」だけを精密検査する（ナローフェーズ）
    for (const auto& result : query_results) {
        int   agent_id  = std::get<1>(result);
        int   car_t     = std::get<2>(result);
        double car_x    = std::get<3>(result);
        double car_y    = std::get<4>(result);

        // 【判定1】「時間」が一致しているかチェック！
        // 歩行者が t=15から16へ動く線分に対し、車の t=15 または t=16 のデータが対象になります
        if (car_t == state_t || car_t == state_t + 1) {
            
            // 【判定2】時間が同じ（近い）なら、線分と点（車の中心）の最短距離を計算
            Segment2D ped_line(Point2D(start_x, start_y), Point2D(end_x, end_y));
            Point2D   car_point(car_x, car_y);
            
            // Boost.Geometryの便利な機能：線分と点の最短距離を一発で計算
            double distance = bg::distance(car_point, ped_line);

            // 車と歩行者の距離が「車のマージン + 歩行者の半径」より近ければ衝突とみなす
            double safe_distance = 1.0 + ped_radius; 

            if (distance < safe_distance) {
              // 今回の衝突の「エージェントID」と「時間」のペアを作成
                std::pair<int, int> collision_key = std::make_pair(agent_id, car_t);

                if (reported_collisions.find(collision_key) == reported_collisions.end()) {
                std::cout << "🚨 【衝突検知】エージェント " << agent_id 
                          << " が時間 t=" << car_t 
                          << " に歩行者の軌跡と衝突します！ (最短距離: " << distance << "m)" << std::endl;
                collision_detected = true;
                reported_collisions.insert(collision_key); // 報告済みとして記録
                }
                // ★将来的に、ここで Online Path Repair（再計算）のフラグを立てます
            }
          
        }
    }
}

if (!collision_detected) {
    std::cout << "✅ 動く歩行者の軌跡（線形）との衝突は予測されませんでした。安全です。" << std::endl;
}
std::cout << "========================================================\n" << std::endl;

// === 追加箇所③：経路リストの中身を確認するテストコードを追加 ===
    std::cout << "\n=== エージェント0（1台目）の経路データを確認 ===" << std::endl;
    // solution[0].states がエージェント0の経路リスト（配列）です
    for (const auto& state_pair : solution[0].states) {
        // ペアの中から、State（状態）だけを取り出す
        State s = state_pair.first;
        // ターミナルに 時間、X座標、Y座標、角度 を出力する
        std::cout << "時刻 t=" << s.time 
                  << " -> 座標(x: " << s.x 
                  << ", y: " << s.y 
                  << ", 角度: " << s.yaw << ")" << std::endl;
    }
    std::cout << "==============================================\n" << std::endl;
    // === 追加箇所③ ===

    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      // std::cout << "Solution for: " << a << std::endl;
      // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
      //   std::cout << solution[a].states[i].second << ": "
      //             << solution[a].states[i].first << "->"
      //             << solution[a].actions[i].first
      //             << "(cost: " << solution[a].actions[i].second << ")"
      //             << std::endl;
      // }
      // std::cout << solution[a].states.back().second << ": "
      //           << solution[a].states.back().first << std::endl;

      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      yaw: " << state.first.yaw << std::endl
            << "      t: " << state.first.time << std::endl;
      }
    }
  } else {
    std::cout << "\033[1m\033[31m Fail to find paths \033[0m\n";
  }
}
