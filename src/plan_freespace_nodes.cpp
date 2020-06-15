#include <intelligent_process_bt/plan_freespace_nodes.h>

PlanFreespace::PlanFreespaceManager::PlanFreespaceManager(const PlanFreespace::PlanRequest& req)
  : mt_rand(std::mt19937(std::random_device{}()))
  , req_(req)
  , result_valid_(false)
{
  auto coin_flip = std::bind(std::uniform_int_distribution<int>(0, 1), mt_rand);
  CAN_PLAN_INTERP = coin_flip();
  CAN_PLAN_TRAJOPT = coin_flip();
  CAN_PLAN_OMPL = coin_flip();

  std::cout << "CAN_PLAN_INTERP: " << CAN_PLAN_INTERP << std::endl;
  std::cout << "CAN_PLAN_TRAJOPT: " << CAN_PLAN_TRAJOPT << std::endl;
  std::cout << "CAN_PLAN_OMPL: " << CAN_PLAN_OMPL << std::endl;
}

//BT_REGISTER_NODES(factory)
//{
//    PlanFreespace::PlanFreespaceManager::RegisterNodes(factory, );
//}

BT::NodeStatus PlanFreespace::PlanFreespaceManager::planJointInterp()
{
  std::cout << "Trying to solve as joint interpolated trajectory..." << std::endl;
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

  if (CAN_PLAN_INTERP)
  {
    res_.traj.resize(5);
    std::iota(res_.traj.begin(), res_.traj.end(), 1.0);
    result_valid_ = true;
    std::cout << "Joint interpolation succeeded" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  std::cout << "Joint interpolation failed" << std::endl;
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PlanFreespace::PlanFreespaceManager::planTrajOpt()
{
  std::cout << "Trying to solve using TrajOpt optimization..." << std::endl;
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

  if (CAN_PLAN_TRAJOPT)
  {
    res_.traj.resize(10);
    std::iota(res_.traj.begin(), res_.traj.end(), 1.0);
    result_valid_ = true;
    std::cout << "TrajOpt succeeded" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  std::cout << "TrajOpt failed" << std::endl;
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PlanFreespace::PlanFreespaceManager::planOMPL()
{
  std::cout << "Trying to solve using OMPL RRTConnect..." << std::endl;
  std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

  if (CAN_PLAN_OMPL)
  {
    res_.traj.resize(15);
    std::iota(res_.traj.begin(), res_.traj.end(), 1.0);
    result_valid_ = true;
    std::cout << "OMPL succeeded" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  std::cout << "OMPL failed" << std::endl;
  return BT::NodeStatus::FAILURE;
}

PlanFreespace::PlanResult PlanFreespace::PlanFreespaceManager::getResult()
{
  if(!result_valid_) throw std::runtime_error("Result not valid");

  return res_;
}

// Register at once all the Actions and Conditions in this file
void PlanFreespace::PlanFreespaceManager::RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerSimpleAction("PlanJointInterp", std::bind(&PlanFreespace::PlanFreespaceManager::planJointInterp, this));
    factory.registerSimpleAction("PlanTrajOpt", std::bind(&PlanFreespace::PlanFreespaceManager::planTrajOpt, this));
    factory.registerSimpleAction("PlanOMPL", std::bind(&PlanFreespace::PlanFreespaceManager::planOMPL, this));
}
