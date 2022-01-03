/**
 * @file sh_astar.cpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief The implement of Spatiotemporal Hybrid-State Astar for single_agent
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <math.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <boost/functional/hash.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
typedef ompl::base::SE2StateSpace::StateType OmplState;

#include "hybrid_astar.hpp"
#include "timer.hpp"

using libMultiRobotPlanning::HybridAStar;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace libMultiRobotPlanning;

namespace Constants {
// [m] --- The minimum turning radius of the vehicle
static const float r = 3;
static const float deltat = 6.75 / 180.0 * M_PI;
// [#] --- A movement cost penalty for turning (choosing non straight motion
// primitives)
static const float penaltyTurning = 1.3;
// [#] --- A movement cost penalty for reversing (choosing motion primitives >
// 2)
static const float penaltyReversing = 2.0;
// [#] --- A movement cost penalty for change of direction (changing from
// primitives < 3 to primitives > 2)
static const float penaltyCOD = 2.0;
// map resolution
static const float mapResolution = 2.0;
static const float xyResolution = r * deltat;
static const float yawResolution = deltat;

// width of car
static const float carWidth = 2.0;
// distance from rear to vehicle front end
static const float LF = 2.0;
// distance from rear to vehicle back end
static const float LB = 1.0;
// obstacle default radius
static const float obsRadius = 1;

// R = 3, 6.75 DEG
const double dx[] = {r * deltat, r* sin(deltat),  r* sin(deltat),
                     -r* deltat, -r* sin(deltat), -r* sin(deltat)};
const double dy[] = {0, -r*(1 - cos(deltat)), r*(1 - cos(deltat)),
                     0, -r*(1 - cos(deltat)), r*(1 - cos(deltat))};
const double dyaw[] = {0, deltat, -deltat, 0, -deltat, deltat};

static inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
}
}  // namespace Constants

struct State {
  State(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
  State(const State&) = default;
  State(State&&) = default;
  State& operator=(const State&) = default;
  State& operator=(State&&) = default;

  bool operator==(const State& other) const {
    return std::tie(x, y, yaw) == std::tie(other.x, other.y, other.yaw);
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")";
  }

  double x;
  double y;
  double yaw;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.yaw);
    return seed;
  }
};
}  // namespace std

using Action = int;  // Action < 6

class Environment {
 public:
  Environment(size_t maxx, size_t maxy, std::unordered_set<State> obstacles,
              State goal)
      : m_obstacles(std::move(obstacles)),
        m_goal(goal)  // NOLINT
  {
    m_dimx = static_cast<int>(maxx / Constants::mapResolution);
    m_dimy = static_cast<int>(maxy / Constants::mapResolution);
    // std::cout << "env build " << m_dimx << " " << m_dimy << " "
    //           << m_obstacles.size() << std::endl;
    holonomic_cost_map = std::vector<std::vector<double>>(
        m_dimx, std::vector<double>(m_dimy, 0));
    m_goal = State(goal.x, goal.y, Constants::normalizeHeadingRad(goal.yaw));
    updateCostmap();
  }

  struct compare_node {
    bool operator()(const std::pair<State, double>& n1,
                    const std::pair<State, double>& n2) const {
      return (n1.second > n2.second);
    }
  };

  uint64_t calcIndex(const State& s) {
    return (uint64_t)(Constants::normalizeHeadingRad(s.yaw) /
                      Constants::yawResolution) *
               (m_dimx * Constants::mapResolution / Constants::xyResolution) *
               (m_dimy * Constants::mapResolution / Constants::xyResolution) +
           (uint64_t)(s.y / Constants::xyResolution) *
               (m_dimx * Constants::mapResolution / Constants::xyResolution) +
           (uint64_t)(s.x / Constants::xyResolution);
  }

  double admissibleHeuristic(const State& s) {
    // non-holonomic-without-obstacles heuristic: use a Reeds-Shepp
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    OmplState* rsStart = (OmplState*)reedsSheppPath.allocState();
    OmplState* rsEnd = (OmplState*)reedsSheppPath.allocState();
    rsStart->setXY(s.x, s.y);
    rsStart->setYaw(s.yaw);
    rsEnd->setXY(m_goal.x, m_goal.y);
    rsEnd->setYaw(m_goal.yaw);
    double reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    // std::cout << "ReedsShepps cost:" << reedsSheppCost << std::endl;
    // Euclidean distance
    double euclideanCost =
        sqrt(pow(m_goal.x - s.x, 2) + pow(m_goal.y - s.y, 2));
    // std::cout << "Euclidean cost:" << euclideanCost << std::endl;
    // holonomic-with-obstacles heuristic
    double twoDoffset = sqrt(pow((s.x - static_cast<int>(s.x)) -
                                     (m_goal.x - static_cast<int>(m_goal.x)),
                                 2) +
                             pow((s.y - static_cast<int>(s.y)) -
                                     (m_goal.y - static_cast<int>(m_goal.y)),
                                 2));
    double twoDCost =
        holonomic_cost_map[static_cast<int>(s.x / Constants::mapResolution)]
                          [static_cast<int>(s.y / Constants::mapResolution)] -
        twoDoffset;
    // std::cout << "holonomic cost:" << twoDCost << std::endl;

    return std::max({reedsSheppCost, euclideanCost, twoDCost});
  }

  bool isSolution(
      const State& state, double gscore,
      std::unordered_map<State, std::tuple<State, Action, double, double>,
                         std::hash<State>>& _camefrom) {
    double goal_distance =
        sqrt(pow(state.x - m_goal.x, 2) + pow(state.y - m_goal.y, 2));
    if (goal_distance > 2 * (Constants::LB + Constants::LF)) return false;

    ompl::base::ReedsSheppStateSpace reedsSheppSpace(Constants::r);
    OmplState* rsStart = (OmplState*)reedsSheppSpace.allocState();
    OmplState* rsEnd = (OmplState*)reedsSheppSpace.allocState();
    rsStart->setXY(state.x, state.y);
    rsStart->setYaw(-state.yaw);
    rsEnd->setXY(m_goal.x, m_goal.y);
    rsEnd->setYaw(-m_goal.yaw);
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath reedsShepppath =
        reedsSheppSpace.reedsShepp(rsStart, rsEnd);

    std::vector<State> path;
    std::unordered_map<State, std::tuple<State, Action, double, double>,
                       std::hash<State>>
        cameFrom;
    cameFrom.clear();
    path.emplace_back(state);
    for (auto pathidx = 0; pathidx < 5; pathidx++) {
      if (fabs(reedsShepppath.length_[pathidx]) < 1e-6) continue;
      double deltat, dx, act, cost;
      switch (reedsShepppath.type_[pathidx]) {
        case 0:  // RS_NOP
          continue;
          break;
        case 1:  // RS_LEFT
          deltat = -reedsShepppath.length_[pathidx];
          dx = Constants::r * sin(-deltat);
          // dy = Constants::r * (1 - cos(-deltat));
          act = 2;
          cost = reedsShepppath.length_[pathidx] * Constants::r *
                 Constants::penaltyTurning;
          break;
        case 2:  // RS_STRAIGHT
          deltat = 0;
          dx = reedsShepppath.length_[pathidx] * Constants::r;
          // dy = 0;
          act = 0;
          cost = dx;
          break;
        case 3:  // RS_RIGHT
          deltat = reedsShepppath.length_[pathidx];
          dx = Constants::r * sin(deltat);
          // dy = -Constants::r * (1 - cos(deltat));
          act = 1;
          cost = reedsShepppath.length_[pathidx] * Constants::r *
                 Constants::penaltyTurning;
          break;
        default:
          std::cout << "\033[1m\033[31m"
                    << "Warning: Receive unknown ReedsSheppPath type"
                    << "\033[0m\n";
          break;
      }
      if (cost < 0) {
        cost = -cost * Constants::penaltyReversing;
        act = act + 3;
      }
      State s = path.back();
      std::vector<std::pair<State, double>> next_path =
          generatePath(s, act, deltat, dx);
      // State next_s(s.x + dx * cos(-s.yaw) - dy * sin(-s.yaw),
      //              s.y + dx * sin(-s.yaw) + dy * cos(-s.yaw),
      //              Constants::normalizeHeadingRad(s.yaw + deltat));
      for (auto iter = next_path.begin(); iter != next_path.end(); iter++) {
        State next_s = iter->first;
        if (!stateValid(next_s))
          return false;
        else {
          gscore += iter->second;
          if (!(next_s == path.back())) {
            cameFrom.insert(std::make_pair<>(
                next_s,
                std::make_tuple<>(path.back(), act, iter->second, gscore)));
          }
          path.emplace_back(next_s);
        }
      }
    }

    m_goal = path.back();
    // auto iter = cameFrom.find(getGoal());
    // do {
    //   std::cout << " From " << std::get<0>(iter->second)
    //             << " to Node:" << iter->first
    //             << " with ACTION: " << std::get<1>(iter->second) << " cost "
    //             << std::get<2>(iter->second) << " g_score "
    //             << std::get<3>(iter->second) << std::endl;
    //   iter = cameFrom.find(std::get<0>(iter->second));
    // } while (calcIndex(std::get<0>(iter->second)) != calcIndex(state));
    // std::cout << " From " << std::get<0>(iter->second)
    //           << " to Node:" << iter->first
    //           << " with ACTION: " << std::get<1>(iter->second) << " cost "
    //           << std::get<2>(iter->second) << " g_score "
    //           << std::get<3>(iter->second) << std::endl;

    _camefrom.insert(cameFrom.begin(), cameFrom.end());
    return true;
  }

  void getNeighbors(const State& s, Action action,
                    std::vector<Neighbor<State, Action, double>>& neighbors) {
    neighbors.clear();
    for (Action act = 0; act < 6; act++) {  // has 6 directions for Reeds-Shepp
      double xSucc, ySucc, yawSucc;
      double g = Constants::dx[0];
      xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
              Constants::dy[act] * sin(-s.yaw);
      ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
              Constants::dy[act] * cos(-s.yaw);
      yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
      if (act != action) {  // penalize turning
        g = g * Constants::penaltyTurning;
        if (act >= 3)  // penalize change of direction
          g = g * Constants::penaltyCOD;
      }
      if (act > 3) {  // backwards
        g = g * Constants::penaltyReversing;
      }
      State tempState(xSucc, ySucc, yawSucc);
      if (stateValid(tempState)) {
        neighbors.emplace_back(
            Neighbor<State, Action, double>(tempState, act, g));
      }
    }
  }

  void onExpandNode(const State& s, int /*fScore*/, int /*gScore*/) {
    Ecount++;
    // std::cout << "Expand " << Ecount << " new Node:" << s << std::endl;
  }

  void onDiscover(const State& s, double fScore, double gScore) {
    Dcount++;
    // std::cout << "Discover " << Dcount << "  Node:" << s << " f:" << fScore
    //           << " g:" << gScore << std::endl;
  }

 public:
  State getGoal() { return m_goal; }
  int Ecount = 0;
  int Dcount = 0;

 private:
  bool stateValid(const State& s) {
    double x_ind = s.x / Constants::mapResolution;
    double y_ind = s.y / Constants::mapResolution;
    if (x_ind < 0 || x_ind >= m_dimx || y_ind < 0 || y_ind >= m_dimy)
      return false;

    Eigen::Matrix2f rot;
    rot << cos(-s.yaw), -sin(-s.yaw), sin(-s.yaw), cos(-s.yaw);
    for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++) {
      Eigen::Matrix<float, 1, 2> obs;
      obs << it->x - s.x, it->y - s.y;
      auto rotated_obs = obs * rot;
      if (rotated_obs(0) > -Constants::LB - Constants::obsRadius &&
          rotated_obs(0) < Constants::LF + Constants::obsRadius &&
          rotated_obs(1) > -Constants::carWidth / 2.0 - Constants::obsRadius &&
          rotated_obs(1) < Constants::carWidth / 2.0 + Constants::obsRadius)
        return false;
    }
    return true;
    // Eigen::Matrix2f rot;
    // double yaw = M_PI / 2;
    // rot << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
    // Eigen::Matrix<float, 1, 2> temp;
    // temp << 1, 2;
    // auto ro = temp * rot;
    // std::cout << ro(0) << ro(1) << std::endl;
  }

  void updateCostmap() {
    boost::heap::fibonacci_heap<std::pair<State, double>,
                                boost::heap::compare<compare_node>>
        heap;
    heap.clear();

    std::set<std::pair<int, int>> temp_obs_set;
    for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++) {
      temp_obs_set.insert(
          std::make_pair(static_cast<int>(it->x / Constants::mapResolution),
                         static_cast<int>(it->y / Constants::mapResolution)));
    }

    int goal_x = static_cast<int>(m_goal.x / Constants::mapResolution);
    int goal_y = static_cast<int>(m_goal.y / Constants::mapResolution);
    heap.push(std::make_pair(State(goal_x, goal_y, 0), 0));

    while (!heap.empty()) {
      std::pair<State, double> node = heap.top();
      heap.pop();

      int x = node.first.x;
      int y = node.first.y;
      for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0) continue;
          int new_x = x + dx;
          int new_y = y + dy;
          if (new_x == goal_x && new_y == goal_y) continue;
          if (new_x >= 0 && new_x < m_dimx && new_y >= 0 && new_y < m_dimy &&
              holonomic_cost_map[new_x][new_y] == 0 &&
              temp_obs_set.find(std::make_pair(new_x, new_y)) ==
                  temp_obs_set.end()) {
            holonomic_cost_map[new_x][new_y] =
                holonomic_cost_map[x][y] +
                sqrt(pow(dx * Constants::mapResolution, 2) +
                     pow(dy * Constants::mapResolution, 2));
            heap.push(std::make_pair(State(new_x, new_y, 0),
                                     holonomic_cost_map[new_x][new_y]));
          }
        }
    }

    // for (size_t i = 0; i < m_dimx; i++) {
    //   for (size_t j = 0; j < m_dimy; j++)
    //     std::cout << holonomic_cost_map[i][j] << "\t";
    //   std::cout << std::endl;
    // }
  }

  std::vector<std::pair<State, double>> generatePath(State startState, int act,
                                                     double deltaSteer,
                                                     double deltaLength) {
    std::vector<std::pair<State, double>> result;
    double xSucc, ySucc, yawSucc, dx, dy, dyaw, ratio;
    result.emplace_back(std::make_pair<>(startState, 0));
    if (act == 0 || act == 3) {
      for (size_t i = 0; i < (size_t)(deltaLength / Constants::dx[act]); i++) {
        State s = result.back().first;
        xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                Constants::dy[act] * sin(-s.yaw);
        ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                Constants::dy[act] * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
        result.emplace_back(
            std::make_pair<>(State(xSucc, ySucc, yawSucc), Constants::dx[0]));
      }
      ratio =
          (deltaLength - static_cast<int>(deltaLength / Constants::dx[act]) *
                             Constants::dx[act]) /
          Constants::dx[act];
      dyaw = 0;
      dx = ratio * Constants::dx[act];
      dy = 0;
    } else {
      for (size_t i = 0; i < (size_t)(deltaSteer / Constants::dyaw[act]); i++) {
        State s = result.back().first;
        xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                Constants::dy[act] * sin(-s.yaw);
        ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                Constants::dy[act] * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
        result.emplace_back(
            std::make_pair<>(State(xSucc, ySucc, yawSucc),
                             Constants::dx[0] * Constants::penaltyTurning));
      }
      ratio =
          (deltaSteer - static_cast<int>(deltaSteer / Constants::dyaw[act]) *
                            Constants::dyaw[act]) /
          Constants::dyaw[act];
      dyaw = ratio * Constants::dyaw[act];
      dx = Constants::r * sin(dyaw);
      dy = -Constants::r * (1 - cos(dyaw));
      if (act == 2 || act == 5) {
        dx = -dx;
        dy = -dy;
      }
    }
    State s = result.back().first;
    xSucc = s.x + dx * cos(-s.yaw) - dy * sin(-s.yaw);
    ySucc = s.y + dx * sin(-s.yaw) + dy * cos(-s.yaw);
    yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw);
    result.emplace_back(std::make_pair<>(State(xSucc, ySucc, yawSucc),
                                         ratio * Constants::dx[0]));
    // std::cout << "Have generate " << result.size() << " path segments:\n\t";
    // for (auto iter = result.begin(); iter != result.end(); iter++)
    //   std::cout << iter->first << ":" << iter->second << "->";
    // std::cout << std::endl;

    return result;
  }

  int m_dimx;
  int m_dimy;
  std::unordered_set<State> m_obstacles;
  std::vector<std::vector<double>> holonomic_cost_map;
  State m_goal;
};

int main() {
  // TODO: read map info from yaml
  std::unordered_set<State> obs;
  obs.insert(State(6.66799, 9.66868, 0));
  obs.insert(State(6.86099, 6.20241, 0));
  obs.insert(State(6.2493, 3.45836, 0));
  obs.insert(State(6.93504, 0.747862, 0));
  obs.insert(State(6.81566, 4.75936, 0));
  State goal(13, 10, -M_PI);
  State start(2, 2, 0);
  Environment env(16, 16, obs, goal);
  HybridAStar<State, Action, double, Environment> hybridAStar(env);
  PlanResult<State, Action, double> solution;
  Timer timer;
  bool searchSuccess = hybridAStar.search(start, solution);
  timer.stop();

  std::string outputFile = "output_h.yaml";
  std::ofstream out(outputFile);
  if (searchSuccess) {
    std::cout << "\033[1m\033[32m Succesfully find a path! \033[0m\n";
    std::cout << "Solution get " << solution.states.size() << " states "
              << solution.actions.size() << " moves\n"
              << "Runtime: " << timer.elapsedSeconds() << std::endl;

    out << "schedule:" << std::endl;
    out << "  agent1:" << std::endl;
    for (size_t i = 0; i < solution.states.size(); ++i) {
      out << "    - x: " << solution.states[i].first.x << std::endl
          << "      y: " << solution.states[i].first.y << std::endl
          << "      yaw: " << solution.states[i].first.yaw << std::endl
          << "      t: " << i << std::endl;
    }
    // for (auto iter = solution.states.begin(); iter != solution.states.end();
    //      iter++)
    //   std::cout << iter->first << ":" << iter->second << "->";
    // std::cout << std::endl;
    // for (auto iter = solution.actions.begin(); iter !=
    // solution.actions.end();
    //      iter++)
    //   std::cout << iter->first << ":" << iter->second << "->";
    // std::cout << std::endl;
    std::cout << "Solution: gscore/cost:" << solution.cost
              << "\t fmin:" << solution.fmin << "\n\rDiscover " << env.Dcount
              << " Nodes and Expand " << env.Ecount << " Nodes." << std::endl;
  } else {
    std::cout << "\033[1m\033[31m Fail to find a path \033[0m\n";
  }
}