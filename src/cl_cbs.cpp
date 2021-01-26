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
