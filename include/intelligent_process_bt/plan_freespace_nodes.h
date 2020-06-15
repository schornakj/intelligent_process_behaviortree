#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <thread>
#include <random>

//using namespace BT;

namespace PlanFreespace
{

struct PlanRequest
{
  std::double_t start, end;
};

struct PlanResult
{
  std::vector<std::double_t> traj;
};

class PlanFreespaceManager
{
public:
  PlanFreespaceManager(const PlanFreespace::PlanRequest& req);

  void RegisterNodes(BT::BehaviorTreeFactory& factory);

  PlanResult getResult();

protected:
  BT::NodeStatus planJointInterp();

  BT::NodeStatus planTrajOpt();

  BT::NodeStatus planOMPL();

  PlanRequest req_;

  bool result_valid_;
  PlanResult res_;

  std::mt19937 mt_rand;

  bool CAN_PLAN_INTERP, CAN_PLAN_TRAJOPT, CAN_PLAN_OMPL;
};
}
