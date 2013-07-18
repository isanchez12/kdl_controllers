
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013 Johns Hopkins University
 *  All rights reserved.
 *
 *  edistribution and use in source and binary forms, with or without
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

#include <controllers/inverse_dynamics_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace kdl_controllers  {

  InverseDynamicsController::InverseDynamicsController() : 
    loop_count_(0)
  { }

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
  
    std::string root_link;
    if(!n.getParam("root_link", root_link)) {
      ROS_ERROR("No root_link given (namespace:%s)", n.getNamespace().c_str());
      return false;
    }

    std::string tip_link;
    if(!n.getParam("tip_link", tip_link)) {
      ROS_ERROR("No tip_link given (namespace:%s)", n.getNamespace().c_str());
      return false;
    }   

    control_toolbox::Pid pid;
    if (!pid.init(ros::NodeHandle(n, "pid")))
      return false;
    //********************modified code  ******************************
    if (!chain_.init(robot, root_link, tip_link))
    {
      ROS_ERROR("MyCartController could not use the chain from '%s' to '%s'",
                root_link.c_str(), tip_link.c_str());
      return false;
    }

    chain_.toKDL(kdl_chain_);
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    //resize the vars in non-realtime
    q_.resize(kdl_chain_.getNrOfJoints());
    q0_.resize(kdl_chain_.getNrOfJoints());
    qdot_.resize(kdl_chain_.getNrOfJoints());

  //  return init(robot, joint_name, pid);
     return true;
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
////*****************modified code **************************************
   /* // Initialize kinematics (KDL tree, KDL chain, and #DOF)
    if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
          robot_description_, root_link_, tip_link_,
          n_dof_, kdl_chain_, kdl_tree_, urdf))
     {
      ROS_ERROR("Could not initialize robot kinematics!");
      //return false;
      }

    // Create inverse dynamics chainsolver
     id_solver_.reset(
        new KDL::ChainIdSolver_RNE(
           kdl_chain_,
           KDL::Vector(gravity_[0],gravity_[1],gravity_[2])));
  */
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
     
/**
   // **************************modified code *******************************8
    KDL::JntArray q_;     //joint positions
    KDL::JntArray q0_;    //joint initial positions
    KDL::JntArray qdot_;  //Joint velocities


    //Get the current Joint positions and velocities
     chain_.getPositions(q_);
     chain_.getVelocities(qdot_);
*/  
    ////Set the computed torque from the gravity compensation library
    //for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++) 
    //{
      //joint_.setCommand( torques_(i) );  
    //}
  
  /*
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
*/
    //for( unsigned int i=0; i <numJoints.get(); i++)
    //{
       //jntarr(i) = JointPoses.Get()[i];
       //if (iksolver -> CartToJnt( jntarr, DesiredTwist.Get(), qdot) >= 0)
       //{
         //for( unsigned int i=0; i <numJoints.get(); i++)
         //{
            //v[i] = qdot(i);
            //JointVelocities.Set(v);
         //}
       //}
    //}


  }

  void InverseDynamicsController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    setCommand(msg->data);
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( kdl_controllers::InverseDynamicsController, controller_interface::ControllerBase)
