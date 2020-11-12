/**
 * @file cl_cbs.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief CL-CBS header
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <chrono>
#include <map>

#include "hybrid_astar.hpp"

#define MAX_RUNTIME 120

namespace libMultiRobotPlanning {
/*!
\example cl_cbs.cpp Implementation of whole CL-CBS method
*/

/*! \brief Car-Like Conflict-Based Search(CL-CBS) algorithm to solve the
Multi-Agent Path Finding for Car-Like robots (CL-MAPF) problem

It applies a body conflict tree to address collisions considering shapes of
the agents. It uses a new algorithm called Spatiotemporal Hybrid-State A* as the
single-agent path planner to generate path satisfying both kinematic and
spatiotemporal constraints. The file also integrates a sequential planning
version of CL-CBS method for the sake of efficiency.

Details of the algorithm can be found in the
following paper:\n
Licheng Wen, Zhen Zhang, Zhe Chen, Xiangrui Zhao, and Yong Liu. CL-MAPF:
Multi-Agent Path Finding for Car-Like Robots with Kinematic and Spatiotemporal
Constraints.[[arxiv](https://arxiv.org/abs/2011.00441)]



\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Conflict Custom conflict description. A conflict needs to be able to be
transformed into a constraint.
\tparam Constraints Custom constraint description. The Environment needs to be
able to search on the low-level while taking the constraints into account.
\tparam Environment This class needs to provide the custom logic. In particular,
it needs to support the following functions:
  - `void setLowLevelContext(size_t agentIdx, const Constraints* constraints)`\n
    Set the current context to a particular agent with the given set of
constraints

  - `Cost admissibleHeuristic(const State& s)`\n
    Admissible heuristic. Needs to take current context into account.

  - `bool isSolution(const State& s, Cost gscore,
      std::unordered_map<State, std::tuple<State, Action, Cost, Cost>`,
                        StateHasher>& cameFrom)\n
     Return true if the given state is a goal state for the current agent. Add
analatic expansion into camefrom struct.

  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int>
>& neighbors)`\n
    Fill the list of neighboring state for the given state s and the current
agent.

  - `bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >&
solution, Conflict& result)`\n
    Finds the first body conflict for the given solution for each agent. Return
true if a conflict conflict was found and false otherwise.

  - `void createConstraintsFromConflict(const Conflict& conflict,
std::map<size_t, Constraints>& constraints)`\n
    Create a list of constraints for the given conflict.

  - `void onExpandHighLevelNode(Cost cost)`\n
    This function is called on every high-level expansion and can be used for
statistical purposes.

  - `void onExpandLowLevelNode(const State& s, Cost fScore, Cost gScore)`\n
    This function is called on every low-level expansion and can be used for
statistical purposes.
*/

template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class CL_CBS {
 public:
  CL_CBS(Environment& environment) : m_env(environment) {}
  ~CL_CBS() {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost>>& solution) {
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;

    for (size_t i = 0; i < initialStates.size(); ++i) {
      // if (i < solution.size() && solution[i].states.size() > 1) {
      //   start.solution[i] = solution[i];
      //   std::cout << "use existing solution for agent: " << i << std::endl;
      // } else {
      LowLevelEnvironment llenv(m_env, i, start.constraints[i]);
      LowLevelSearch_t lowLevel(llenv);
      bool success = lowLevel.search(initialStates[i], start.solution[i]);
      if (!success) {
        return false;
      }
      // }
      start.cost += start.solution[i].cost;
    }

    // std::priority_queue<HighLevelNode> open;
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>
        open;

    auto handle = open.push(start);
    (*handle).handle = handle;

    std::chrono::high_resolution_clock::time_point
        startTime = std::chrono::high_resolution_clock::now(),
        endTime;
    solution.clear();
    int id = 1;
    while (!open.empty()) {
      endTime = std::chrono::high_resolution_clock::now();
      if (std::chrono::duration_cast<std::chrono::duration<double>>(endTime -
                                                                    startTime)
              .count() > MAX_RUNTIME) {
        open.clear();
        std::cout << "\033[1m\033[31m Plan out of runtime time! \033[0m\n";
        return false;
      }

      HighLevelNode P = open.top();
      m_env.onExpandHighLevelNode(P.cost);
      // std::cout << "expand: " << P << std::endl;

      open.pop();

      Conflict conflict;
      if (!m_env.getFirstConflict(P.solution, conflict)) {
        // std::cout << "done; cost: " << P.cost << std::endl;
        solution = P.solution;
        return true;
      }

      // create additional nodes to resolve conflict
      // std::cout << "Found conflict: " << conflict << std::endl;

      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);
      for (const auto& c : constraints) {
        // std::cout << "Add HL node for " << c.first << std::endl;
        size_t i = c.first;
        // std::cout << "create child with id " << id << std::endl;
        HighLevelNode newNode = P;
        newNode.id = id;
        // (optional) check that this constraint was not included already
        // std::cout << newNode.constraints[i] << std::endl;
        // std::cout << c.second << std::endl;
        assert(!newNode.constraints[i].overlap(c.second));

        newNode.constraints[i].add(c.second);

        newNode.cost -= newNode.solution[i].cost;

        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i]);
        LowLevelSearch_t lowLevel(llenv);
        bool success = lowLevel.search(initialStates[i], newNode.solution[i]);

        newNode.cost += newNode.solution[i].cost;

        if (success) {
          // std::cout << "  success. cost: " << newNode.cost << std::endl;
          auto handle = open.push(newNode);
          (*handle).handle = handle;
        }

        ++id;
      }
    }

    return false;
  }

 private:
  struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost>> solution;
    std::vector<Constraints> constraints;

    Cost cost;

    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      // if (cost != n.cost)
      return cost > n.cost;
      // return id > n.id;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
          os << "  " << c.solution[i].states[t].first << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };

  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints)
        : m_env(env)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    bool isSolution(
        const State& s, Cost g,
        std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                           std::hash<State>>& camefrom) {
      return m_env.isSolution(s, g, camefrom);
    }

    void getNeighbors(const State& s, Action act,
                      std::vector<Neighbor<State, Action, Cost>>& neighbors) {
      m_env.getNeighbors(s, act, neighbors);
    }

    State getGoal() { return m_env.getGoal(); }

    int calcIndex(const State& s) { return m_env.calcIndex(s); }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      // std::cout << "LL expand: " << s << std::endl;
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
      // std::cout << "LL discover: " << s << std::endl;
      // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    }

   private:
    Environment& m_env;
  };

 private:
  Environment& m_env;
  typedef HybridAStar<State, Action, Cost, LowLevelEnvironment>
      LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
