
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

// boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_; 

unsigned int num_joints;

namespace kdl_controllers  {

  InverseDynamicsController::InverseDynamicsController(): 
    id_solver_(NULL)
    ,ext_wrenches_()
    ,q_()
    ,qdot_()
    ,accelerations_()
    ,torques_()
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
  /*
    // Get a KDL tree from the robot URDF
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree_)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
    }
    */
    if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
          robot_description_, root_link_, tip_link_,
          n_dof_, kdl_chain_, kdl_tree_, urdf_model))
    {
      ROS_ERROR("Could not initialize robot kinematics!");
      return false;
    }
    
    num_joints = n_dof_;
    ////////////////////GETTING JOINTS ///////////////////////
//     boost::shared_ptr<const urdf::Link> link = urdf_model.getLink(tip_link_);
 /*   std::string link = tip_link_;

    std::vector<std::string> JOINT_ ;

    for(unsigned int i= 0; i <= n_dof_; i++)
    {
      //tip to root
      while ( link != root_link_) 
      {

        joint_(i) = urdf_model.getParentJoint(link);

        if (!joint_(i)) 
        {
          ROS_ERROR("Could not find joint: %s", joint_(i).c_str());
          return false;
        }

         link = urdf_model.getParentLink(tip_link_);
      }
    }
*/
/*
    //Print out the list of all the joints that will be used
    ROS_INFO("liST OF ALL THE JOINTS THAT WILL BE COMMANDED");
    
    for (unsigned int j=0; j < n_dof_; j++) 
    {
      ROS_INFO("joint  :   %s \n ", joint_(j).c_str());
    }
*/
    /////////////////////////////////////////////////////
   
    // Create inverse dynamics chainsolver
    id_solver_ = new KDL::ChainIdSolver_RNE(
        kdl_chain_,
        KDL::Vector(gravity_[0],gravity_[1],gravity_[2]));

    // Resize working vectors
    q_.resize(n_dof_);   //positions
    qdot_.resize(n_dof_); //velocities
    accelerations_.resize(n_dof_);
    torques_.resize(n_dof_);
    ext_wrenches_.resize(kdl_chain_.getNrOfSegments());

    // Zero out torque data
    torques_.data.setZero();
    accelerations_.data.setZero();

    return true;
  }

  bool InverseDynamicsController::init( hardware_interface::EffortJointInterface *robot, hardware_interface::JointStateInterface* hw, ros::NodeHandle &n)
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

    // get all joint states from the hardware interface
    const std::vector<std::string>& joint_names = hw->getNames();
 
    for (unsigned i=0 ; i < joint_names.size(); i++)
      ROS_DEBUG("Got joint %s", joint_names[i].c_str());

    // get joints handles and joints states
    for (unsigned i=0; i < joint_names.size(); i++)
    {
      joint_handles_[i] = robot -> getHandle(joint_names[i]); 
/*
      joint_states_.push_back( hw -> getHandle(joint_names[i])); 
      name.push_back(joint_names[i]); //name of the joint
      position.push_back(0.0); //storage for the joint's position
      velocity.push_back(0.0); //storage for the joint's velocity
      effort.push_back(0.0);   //storage for the joint's torque
*/
    }


    return true;
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
   //get the positions of all the joints
    for (unsigned i=0; i < num_joints ; i++)
    {
      command_.initRT( joint_handles_[i].getPosition() );

    }
 
  }

  void InverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    double command = *(command_.readFromRT());
    
    ///////////////////COMPUTE POSITION ERROR/////////////////////////////////////////
  /*  //IS  THIS NECESSARY FOR THE KDL NEWTON EULER COMPUTATION????
    if (joint_urdf_->type == urdf::Joint::REVOLUTE)
    {
      angles::shortest_angular_distance_with_limits(joint_.getPosition(),
          command,
          joint_urdf_->limits->lower,
          joint_urdf_->limits->upper,
          error);
    }
    else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
    {
      error = angles::shortest_angular_distance(joint_.getPosition(), command);
    }
    else //prismatic
    {
      error = command - joint_.getPosition();
    }

    // Compute velocity error assuming desired velocity is 0
    vel_error = 0.0 - joint_.getVelocity();

*/
//-----------------------------------------------------------------------------
    // Get the current joint positions and velocities.                                                                                                                              
    for (unsigned int i=0; i < num_joints; i++) {
        
      q_(i)    = joint_handles_[i].getPosition();
      qdot_(i) = joint_handles_[i].getVelocity();

    }
/*
    // Compute inverse dynamics
    // This computes the torques on each joint of the arm as a function of
    // the arm's joint-space position, velocities, accelerations, external
    // forces/torques and gravity.
    if(id_solver_->CartToJnt(
          q_,
          qdot_,
          accelerations_,
          ext_wrenches_,
          torques_) != 0)
    {
      ROS_ERROR("Could not compute joint torques!");
    }
*/
    /*
    // Send joint positions
    torques_out_port_.write( torques_ );

    for (unsigned i=0; i < num_joints ; i++)
    {
      joint_handles_[i].setCommand( torques_ );

    }
    */
  }

  void InverseDynamicsController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    setCommand(msg->data);
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( kdl_controllers::InverseDynamicsController, controller_interface::ControllerBase)
