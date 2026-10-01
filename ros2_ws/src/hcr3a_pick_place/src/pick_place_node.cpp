// Pick-and-place for the HCR-3A with MoveIt 2 (MoveGroupInterface).
//
// home -> above cube -> straight down -> close -> straight up -> above target
//      -> straight down -> open -> straight up -> home
//
// Grasping is real physics in Gazebo (fingertip friction); in the MoveIt planning
// scene the cube is attached to the tcp while carried so the planner accounts for it.

#include <chrono>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("hcr3a_pick_place");

moveit_msgs::msg::CollisionObject makeBox(const std::string & id, double x, double y, double z,
                                          double sx, double sy, double sz)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = "world";
  obj.id = id;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {sx, sy, sz};
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

// tcp z-axis pointing down, fingers closing along world x
geometry_msgs::msg::Pose topDown(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = 1.0;
  p.orientation.y = 0.0;
  p.orientation.z = 0.0;
  p.orientation.w = 0.0;
  return p;
}
}  // namespace

class PickPlace
{
public:
  explicit PickPlace(const rclcpp::Node::SharedPtr & node)
  : node_(node), arm_(node, "arm"), gripper_(node, "gripper")
  {
    arm_.setEndEffectorLink("tcp");
    arm_.setPlanningTime(5.0);
    arm_.setNumPlanningAttempts(5);
    arm_.setMaxVelocityScalingFactor(param("velocity_scaling", 0.3));
    arm_.setMaxAccelerationScalingFactor(param("velocity_scaling", 0.3));
    gripper_.setMaxVelocityScalingFactor(1.0);
    gripper_.setMaxAccelerationScalingFactor(1.0);
  }

  bool run()
  {
    const double cube_x = param("cube_x", 0.32), cube_y = param("cube_y", 0.18);
    const double cube_z = param("cube_z", 0.12);  // cube center, resting on the table
    const double place_x = param("place_x", 0.32), place_y = param("place_y", -0.18);
    const double place_z = cube_z + 0.002;
    const double hover = param("hover", 0.10);

    setupScene(cube_x, cube_y, cube_z);

    return step("home", [&] { return moveNamed(arm_, "home"); }) &&
           step("open gripper", [&] { return moveNamed(gripper_, "open"); }) &&
           step("move above cube", [&] { return moveTo(topDown(cube_x, cube_y, cube_z + hover)); }) &&
           step("descend", [&] { return moveLinear(topDown(cube_x, cube_y, cube_z)); }) &&
           step("attach + close gripper", [&] {
             arm_.attachObject("cube", "tcp", {"finger1_Link", "finger2_Link", "effector_Link"});
             waitForScene();
             return moveNamed(gripper_, "close");
           }) &&
           step("lift", [&] { return moveLinear(topDown(cube_x, cube_y, cube_z + hover)); }) &&
           step("move above target", [&] { return moveTo(topDown(place_x, place_y, place_z + hover)); }) &&
           step("lower", [&] { return moveLinear(topDown(place_x, place_y, place_z)); }) &&
           step("open gripper + detach", [&] {
             bool ok = moveNamed(gripper_, "open");
             arm_.detachObject("cube");
             waitForScene();
             return ok;
           }) &&
           step("retreat", [&] { return moveLinear(topDown(place_x, place_y, place_z + hover)); }) &&
           step("home", [&] { return moveNamed(arm_, "home"); });
  }

private:
  double param(const std::string & name, double def)
  {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, def);
    }
    return node_->get_parameter(name).as_double();
  }

  template <typename F>
  bool step(const std::string & name, F && fn)
  {
    RCLCPP_INFO(LOGGER, ">> %s", name.c_str());
    if (!fn()) {
      RCLCPP_ERROR(LOGGER, "step failed: %s", name.c_str());
      return false;
    }
    return true;
  }

  void waitForScene() { std::this_thread::sleep_for(std::chrono::milliseconds(500)); }

  void setupScene(double cx, double cy, double cz)
  {
    // Tables and cube mirror hcr3a_gazebo/worlds/pick_place.sdf
    std::vector<moveit_msgs::msg::CollisionObject> objects = {
      makeBox("table_pick", 0.32, 0.18, 0.05, 0.14, 0.14, 0.1),
      makeBox("table_place", 0.32, -0.18, 0.05, 0.14, 0.14, 0.1),
      makeBox("cube", cx, cy, cz, 0.04, 0.04, 0.04),
    };
    scene_.applyCollisionObjects(objects);
    waitForScene();
  }

  bool moveNamed(MoveGroupInterface & group, const std::string & target)
  {
    group.setStartStateToCurrentState();
    group.setNamedTarget(target);
    MoveGroupInterface::Plan plan;
    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return false;
    }
    auto result = group.execute(plan);
    // The gripper stalls on the cube before reaching "close"; that is the intended outcome.
    return result == moveit::core::MoveItErrorCode::SUCCESS || &group == &gripper_;
  }

  bool moveTo(const geometry_msgs::msg::Pose & pose)
  {
    arm_.setStartStateToCurrentState();
    // IK seeded from the current state keeps the arm on the same solution branch,
    // so the following straight-line moves don't need wrist/elbow flips.
    if (!arm_.setJointValueTarget(pose, "tcp")) {
      RCLCPP_ERROR(LOGGER, "no IK solution for target pose");
      return false;
    }
    MoveGroupInterface::Plan plan;
    if (arm_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return false;
    }
    return arm_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // Straight-line tcp motion, slowed down for approach/retreat.
  bool moveLinear(const geometry_msgs::msg::Pose & pose)
  {
    arm_.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory traj;
    // jump_threshold > 0 rejects paths with IK branch flips instead of swinging the arm around
    const double fraction = arm_.computeCartesianPath({pose}, 0.005, 5.0, traj);
    if (fraction < 0.99) {
      RCLCPP_ERROR(LOGGER, "cartesian path only %.0f%% feasible", fraction * 100.0);
      return false;
    }
    robot_trajectory::RobotTrajectory rt(arm_.getRobotModel(), "arm");
    rt.setRobotTrajectoryMsg(*arm_.getCurrentState(), traj);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    totg.computeTimeStamps(rt, 0.1, 0.1);
    rt.getRobotTrajectoryMsg(traj);
    return arm_.execute(traj) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  rclcpp::Node::SharedPtr node_;
  MoveGroupInterface arm_;
  MoveGroupInterface gripper_;
  PlanningSceneInterface scene_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "hcr3a_pick_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });

  bool ok = false;
  {
    PickPlace pick_place(node);
    ok = pick_place.run();
  }
  RCLCPP_INFO(LOGGER, ok ? "pick-and-place finished" : "pick-and-place aborted");

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
