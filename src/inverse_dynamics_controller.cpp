
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

#include <kdl_controllers/inverse_dynamics_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

//#include <terse_roscpp/param.h>
#include <list>

#include <iostream>
#include <map>
#include <urdf/model.h>
#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl_urdf_tools/tools.h>
#include <ros/ros.h>

unsigned int num_joints;

namespace kdl_controllers  {

  InverseDynamicsController::InverseDynamicsController():
     robot_description_("")
    ,root_link_("")
    ,tip_link_("")
    ,gravity_(3, 0.0)
    ,n_dof_(0)
    ,kdl_tree_()
    ,kdl_chain_()
    ,id_solver_(NULL)  
    ,ext_wrenches_()
    ,positions_()
    // ,q_() //positions
    // ,qdot_() //velocities
     ,accelerations_()
    ,torques_()
    {
    }

  InverseDynamicsController::~InverseDynamicsController()
  {
    sub_command_.shutdown();
  }

  bool InverseDynamicsController::init( hardware_interface::EffortJointInterface *robot, 
                                        ros::NodeHandle &n)
  { 
    urdf::Model urdf_model;
    std::string xml_string;

    ROS_INFO("searching for URDF to parse");
    if (!n.getParam("robot_description", robot_description_))
    {
      ROS_ERROR("Failed to find ROBOT_DESCRIPTION FROM THE PARAM SERVER");
      return false;
    }
 
    if (!urdf_model.initString(robot_description_)){

      ROS_FATAL("Could not initialize robot model");
      return false;
    }
    ROS_INFO("FOUND URDF AND WAS SUCCESSFULLY PARSED");

    // Get Root and Tip From Parameter Service
    if (!n.getParam("root_link", root_link_)) {
      ROS_FATAL("EE: No root_link_ found on parameter server");
      return false;
    }

     ROS_INFO("LOADING THE ROOT LINK PLEASE WAIT");

    if (!n.getParam("tip_link", tip_link_)) {
      ROS_FATAL("EE: No root_link_ found on parameter server");
      return false;
    }

    ROS_INFO("LOADING THE TIP LINK PLEASE WAIT");


    ////////////////////////////////////////////////////////////////////////
 
    if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
          robot_description_, root_link_, tip_link_,
          n_dof_, kdl_chain_, kdl_tree_, urdf_model))
    {
      ROS_ERROR("Could not initialize robot kinematics!");
      return false;
    }
    ROS_INFO("SUCCESSFULLY LOADED KDL_URDF_TOOLS");
    
    
    // Create inverse dynamics chainsolver
    id_solver_.reset( new KDL::ChainIdSolver_RNE(   
        kdl_chain_,
        KDL::Vector(gravity_[0],gravity_[1],gravity_[2])));
    
    num_joints = n_dof_;
   
    for (size_t i=0; i<kdl_chain_.getNrOfSegments(); i++){
     
      if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None){ 
    /*
        JointState* jnt = robot_state->getJointState(kdl_chain_.getSegment(i).getJoint().getName());
        if (!jnt){
          ROS_ERROR("Joint '%s' is not found in joint state vector", kdl_chain_.getSegment(i).getJoint().getName().c_str());
          return false;
        }
        joints_.push_back(jnt);
     */ 
      ROS_INFO("SUCCESSFULLY FOULD JOINTS FROM THE KDL CHAIN");
      }
    }
    //ROS_DEBUG("Added %i joints", int(joints_.size()));


    // Resize working vectors
    positions_.resize(n_dof_);   //positions & velocities
    torques_.resize(n_dof_);
    ext_wrenches_.resize(kdl_chain_.getNrOfSegments());

    // Zero out torque data
    torques_.data.setZero();
    accelerations_.data.setZero();
  
   /////////////////////////////////////////////////////////////////
    return true;
  }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool InverseDynamicsController::readJoints(urdf::Model &urdf_model)
{
  // get joint maxs and mins
  boost::shared_ptr<const urdf::Link> link = urdf_model.getLink(tip_link_);
  boost::shared_ptr<const urdf::Joint> joint;
  //root to tip
 /* while (link && link->name != root_link_) {
    joint = urdf_model.getJoint(link->parent_joint->name);
    if (!joint) {
      ROS_ERROR("Could not find joint: ");
      return false;
    }
    link = urdf_model.getLink(link->getParent()->name);
  }
*/
  ROS_INFO("LOADING READJOINTS FUNCTION PLEASE WAIT");

}
/*
//Get the vector of joint Names register to this interface
  std::vector<std::string> InverseDynamicsController::getJointNames()
  {
    
    std::vector<std::string> out;
    out.reserve(handle_map_.size());
    for( HandleMap::const_iterator it = handle_map_.begin(); it != handle_map_.end(); ++it)
    {   
      out.push_back(it->first);
    }   
    return out;
  }
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
  /*
   //get the positions of all the joints
    for (unsigned i=0; i < num_joints ; i++)
    {
      command_.initRT( joint_handles_[i].getPosition() );
    }
 */
  }
  void InverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    double command = *(command_.readFromRT());
  /* 
    // Get the current joint positions and velocities.                                                                                                                              
    for (unsigned int i=0; i < num_joints; i++) 
    {
      positions.q_(i)    = joint_handles_[i].getPosition();
      positions.qdot_(i) = joint_handles_[i].getVelocity();
    }
    // Compute inverse dynamics
    // This computes the torques on each joint of the arm as a function of
    // the arm's joint-space position, velocities, accelerations, external
    // forces/torques and gravity.
 */
     
/*
    if(id_solver_ -> CartToJnt(
          positions_.q_,
          positions_.qdot_,
          accelerations_,
          ext_wrenches_,
          torques_) != 0)
    {
      ROS_ERROR("Could not compute joint torques!");
    }
*/
    /*
    for (uint i = 0; i < num_joints ; i++) 
    {

      ROS_INFO("TORQUE computation: %lf for joint: %d", torques_(i), i);
     // joint_handles_[i].setCommand( torques_(i));
    }
*/
  }

  void InverseDynamicsController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    setCommand(msg->data);
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( kdl_controllers::InverseDynamicsController, controller_interface::ControllerBase)
