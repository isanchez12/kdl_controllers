
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

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl_urdf_tools/tools.h>
#include <ros/ros.h>

boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_(NULL); 

namespace kdl_controllers  {

  InverseDynamicsController::InverseDynamicsController()
  {
  }

  InverseDynamicsController::~InverseDynamicsController()
  {
    sub_command_.shutdown();
  }

  bool InverseDynamicsController::init(
      const std::string& robot_description_, 
      const std::string& root_link_,
      const std::string& tip_link_,
      KDL::Chain& kdl_chain_ ,
      KDL::Tree& kdl_tree_,  
      unsigned int& n_dof_)
  {

    // Initialize kinematics (KDL tree, KDL chain, and #DOF)
    urdf::Model urdf_model;
    // Construct an URDF model from the xml string
    urdf_model.initString(robot_description_);
    // Get a KDL tree from the robot URDF
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree_)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
    }
    
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

    // Zero out torque data
    torques_.data.setZero();
    accelerations_.data.setZero();

    return true;
  }

  bool InverseDynamicsController::init( hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  { 

   // Get URDF XML
    std::string urdf_xml, full_urdf_xml;
    n.param("urdf_xml",urdf_xml,std::string("robot_description"));
    n.searchParam(urdf_xml,full_urdf_xml); 
    ROS_DEBUG("Reading xml file from parameter server");
    std::string result;
   
    if (!n.getParam(full_urdf_xml, result)) {
      ROS_FATAL("Could not load the xml from parameter server: %s", urdf_xml.c_str());
      return false;
    }
     
    // boost::shared_ptr<const urdf::Link> link = urdf_model.getLink(tip_Link_);

    // Get Root and Tip From Parameter Service
    if (!n.getParam("/root_link", root_link_)) {
      ROS_FATAL("EE: No root_link_ found on parameter server");
      return false;
    }

     ROS_INFO("LOADING THE TIP LINK PLEASE WAIT");

    if (!n.getParam("/tip_link", tip_link_)) {
      ROS_FATAL("EE: No root_link_ found on parameter server");
      return false;
    }

    ROS_INFO("LOADING THE TIP LINK PLEASE WAIT");
    
    num_joints = 0;
    // get joint maxs and mins
    boost::shared_ptr<const urdf::Link> link = urdf_model.getLink(tip_link_);
    boost::shared_ptr<const urdf::Joint> joint_urdf;
    
  /* //root to tip
    while (tip_link_urdf_ && tip_link_urdf_ ->name != root_link_) 
    {
      joint_ = urdf_model.getJoint(link->parent_joint->name);
    
      if (!joint_) 
      {
        ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
        return false;
      }
          if (joint_ -> type != urdf::Joint::UNKNOWN && joint_->type != urdf::Joint::FIXED) 
          {
                     
            num_joints++;
            ROS_INFO("The total number of joints is: %d", num_joints);
          }
      
          link = urdf_model.getLink(link->getParent()->name);
    }
*/
    return true;
  }
  
  /*
 
  bool InverseDynamicsController::readLinks(urdf::Model &urdf_model) 
  {
      boost::shared_ptr<const urdf::Link> root_link_urdf_ = urdf_model.getLink(root_link_);
      boost::shared_ptr<const urdf::Link> tip_link_urdf_  = urdf_model.getLink(tip_link_);
  }
*/

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

    /************error with this command**********fix
     * gzserver: /home/isan/M_catkin_ws/src/ros_control/hardware_interface/include/hardware_interface/joint_state_interface.h:70: 
     * double hardware_interface::JointStateHandle::getPosition() const: Assertion `pos_' failed.
     *
     *
     */
      //command_.initRT(joint_.getPosition());
  }



  void InverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    double command = *(command_.readFromRT());

  
    
    //double commanded_effort = pid_controller_.computeCommand(postion, velocity, period); 
    
    
    //  joint_.setCommand( torques_ );

    //for (unsigned int i = 0 ; i < njoints_ ; i++) 
    //{
     //// joint_handles_[i].setCommand( torques_
      //joint_.setCommand( torques_(i) );  
    //}

  }

  void InverseDynamicsController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    setCommand(msg->data);
  }

  /*
  void InverseDynamicsController::constructSolvers() {
  //construck inverse dynamics controller
  id_solver_ = new KDL::ChainIdSolver_RNE(kdl_chain_);
  }
*/
} // namespace

PLUGINLIB_EXPORT_CLASS( kdl_controllers::InverseDynamicsController, controller_interface::ControllerBase)
