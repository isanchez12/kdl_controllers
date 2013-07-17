
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013 Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//#include <iostream>
//#include <map>

//#include <Eigen/Dense>

//#include <kdl/tree.hpp>

//#include <kdl_parser/kdl_parser.hpp>

#include <controllers/inverse_dynamics_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace kdl_controllers  {

  InverseDynamicsController::InverseDynamicsController(std::string const& name ) : 
    TaskContext(name)
    //Properties
    ,loop_count_(0)
    ,robot_description_("")
    ,root_link_("")
    ,tip_link_("")
    ,gravity_(3,0.0)
    // Working variables
    ,n_dof_(0)
    ,kdl_tree_()
    ,kdl_chain_()
    ,id_solver_(NULL)
    ,positions_()
    ,torques_()
  { 
  
    // Declare properties
    this->addProperty("robot_description",robot_description_)
      .doc("The WAM URDF xml string.");
    this->addProperty("gravity",gravity_)
      .doc("The gravity vector in the root link frame.");
    this->addProperty("root_link",root_link_)
      .doc("The root link for the controller.");
    this->addProperty("tip_link",tip_link_)
      .doc("The tip link for the controller.");
    // Configure data ports
    this->ports()->addEventPort("positions_in", positions_in_port_)
      .doc("Input port: nx1 vector of joint positions. (n joints)");
    this->ports()->addPort("torques_out", torques_out_port_)
      .doc("Output port: nx1 vector of joint torques. (n joints)");

    // Initialize properties from rosparam
    rtt_ros_tools::load_rosparam_and_refresh(this);
  
  }

  InverseDynamicsController::~InverseDynamicsController()
  {
    sub_command_.shutdown();
  }

  bool InverseDynamicsController::init(hardware_interface::EffortJointInterface *robot, 
      const std::string &joint_name, const control_toolbox::Pid &pid)
  {
    joint_ = robot->getHandle(joint_name);
    pid_controller_ = pid;

    // get urdf info about joint
    urdf::Model urdf;
    if (!urdf.initParam("robot_description")){
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }
    joint_urdf_ = urdf.getJoint(joint_name);
    if (!joint_urdf_){
      ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
      return false;
    }

    return true;
  }

  bool InverseDynamicsController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    std::string joint_name;
    if (!n.getParam("joint", joint_name)) {
      ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
      return false;
    }

    control_toolbox::Pid pid;
    if (!pid.init(ros::NodeHandle(n, "pid")))
      return false;


    return init(robot, joint_name, pid);
  }
  
  void InverseDynamicsController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
  {
     pid_controller_.setGains(p,i,d,i_max,i_min);
  }

  void InverseDynamicsController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
     pid_controller_.getGains(p,i,d,i_max,i_min);
  }

  std::string InverseDynamicsController::getJointName()
  {
    return joint_.getName();
  }

  // Set the joint position command
  void InverseDynamicsController::setCommand(double cmd)
  {
    // the writeFromNonRT can be used in RT, if you have the guarantee that 
    //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
    //  * there is only one single rt thread
    command_.writeFromNonRT(cmd);
  }


  void InverseDynamicsController::starting(const ros::Time& time) 
  {
    command_.initRT(joint_.getPosition());
    pid_controller_.reset();
//*****************modified code **************************************
    // Initialize kinematics (KDL tree, KDL chain, and #DOF)
    urdf::Model urdf_model;
    if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
          robot_description_, root_link_, tip_link_,
          n_dof_, kdl_chain_, kdl_tree_, urdf_model))
    {
      ROS_ERROR("Could not initialize robot kinematics!");
      return false;
    }
    // Create inverse dynamics chainsolver
    id_solver_.reset(
        new KDL::ChainIdSolver_RNE(
          kdl_chain_,
          KDL::Vector(gravity_[0],gravity_[1],gravity_[2])));
    // Resize working vectors
    positions_.resize(n_dof_);
    accelerations_.resize(n_dof_);
    torques_.resize(n_dof_);
    ext_wrenches_.resize(kdl_chain_.getNrOfSegments());
  
  }


  void InverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    double command = *(command_.readFromRT());

    double error, vel_error;

    // Compute position error
    if (joint_urdf_->type == urdf::Joint::REVOLUTE)
    {
      angles::shortest_angular_distance_with_limits(command,
          joint_.getPosition(), 
          joint_urdf_->limits->lower, 
          joint_urdf_->limits->upper,
          error);
    }
    else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
    {
      error = angles::shortest_angular_distance(command, joint_.getPosition());
    }
    else //prismatic
    {
      error = command - joint_.getPosition();
    }

    // Compute velocity error assuming desired velocity is 0
    vel_error = 0.0 - joint_.getVelocity();

    // Set the PID error and compute the PID command with nonuniform
    // time step size. This also allows the user to pass in a precomputed derivative error. 
    double commanded_effort = pid_controller_.computeCommand(error, vel_error, period); 
    joint_.setCommand( commanded_effort );

    //**************************modified code *******************************8
    //
    ////Set the computed torque from the gravity compensation library
    //for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++) 
    //{
      //joint_.setCommand( torques_(i) );  
    //}
   
    // Read in the current joint positions & velocities
    positions_in_port_.readNewest( positions_ );

    // Compute inverse dynamics
    // This computes the torques on each joint of the arm as a function of
    // the arm's joint-space position, velocities, accelerations, external
    // forces/torques and gravity.
    if(id_solver_->CartToJnt(
          positions_.q,
          positions_.qdot,
      //  accelerations_,
          ext_wrenches_,
          torques_) != 0)

    for( unsigned int i=0; i <numJoints.get(); i++)
    {
       jntarr(i) = JointPoses.Get()[i];
       if (iksolver -> CartToJnt( jntarr, DesiredTwist.Get(), qdot) >= 0)
       {
         for( unsigned int i=0; i <numJoints.get(); i++)
         {
            v[i] = qdot(i);
            JointVelocities.Set(v);
         }
       }
    }


  }

  void InverseDynamicsController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    setCommand(msg->data);
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( kdl_controllers::InverseDynamicsController, controller_interface::ControllerBase)
