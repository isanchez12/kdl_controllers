
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

//boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_(); 

unsigned int num_joints;

namespace kdl_controllers  {

  InverseDynamicsController::InverseDynamicsController(): 
      gravity_(3, 0.0)
     ,n_dof_(0)
     ,kdl_tree_()
     ,kdl_chain_()
     ,id_solver_(NULL)  
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

  bool InverseDynamicsController::init( hardware_interface::EffortJointInterface *robot, 
                                        ros::NodeHandle &n)
                                    // ,hardware_interface::JointStateInterface* hw) 
  { 
    urdf::Model urdf_model;

    if (!urdf_model.initParam("robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file (namespace: %s)", n.getNamespace().c_str());
      return false;
    }
    
    
     std::string my_joint;
         if (!n.getParam("mtm_right_outer_yaw_joint", my_joint)){
                 ROS_ERROR("Could not find joint name");
                       return false;
                           }
         ROS_INFO(" PARAM SERVER HAS FOUND MTM_RIGHT_OUTER_YAW_JOINT");
        
        joint_ = robot-> getHandle(my_joint);

    
    
    
    
    // Get Root and Tip From Parameter Service
    if (!n.getParam("mtm_right_top_panel", root_link_)) {
      ROS_FATAL("EE: No root_link_ found on parameter server");
      return false;
    }

     ROS_INFO("LOADING THE TIP LINK PLEASE WAIT");

    if (!n.getParam("/mtm_right_wrist_roll_link", tip_link_)) {
      ROS_FATAL("EE: No root_link_ found on parameter server");
      return false;
    }

    ROS_INFO("LOADING THE TIP LINK PLEASE WAIT");
///////////////////////////////////////////////////////////////////////////
    
    if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
          robot_description_, root_link_, tip_link_,
          n_dof_, kdl_chain_, kdl_tree_, urdf_model))
    {
      ROS_ERROR("Could not initialize robot kinematics!");
      return false;
    }
    
    num_joints = n_dof_;
   
    // Create inverse dynamics chainsolver
    id_solver_.reset( new KDL::ChainIdSolver_RNE(   
        kdl_chain_,
        KDL::Vector(gravity_[0],gravity_[1],gravity_[2])));

    // Resize working vectors
    q_.resize(n_dof_);   //positions
    qdot_.resize(n_dof_); //velocities
    accelerations_.resize(n_dof_);
    torques_.resize(n_dof_);
    ext_wrenches_.resize(kdl_chain_.getNrOfSegments());

    // Zero out torque data
    torques_.data.setZero();
    accelerations_.data.setZero();
    
    //////////////////////////////////////////////////////////////////////
    // get all joint states from the hardware interface
    
    ROS_INFO("GETTING JOINT HANDLES FROM HARDWARE INTERFACE");
    boost::shared_ptr<const urdf::Link> link = urdf_model.getLink(tip_link_);
    boost::shared_ptr<const urdf::Joint> joint_n_ ;
   
    // Get joint handle from hardware interface
  //  joint_ = robot->getHandle(joint_name);

   //----------------------------------------------------------------- 
    for(unsigned int i= 0; i < n_dof_; i++)
    {
      while ( tip_link_ != root_link_) 
      {
       
        /*joint_n_ = urdf_model.getJoint(link);
        
        joint_handles_[i] = robot -> getHandle(joint_n_.getJointName()); 
        
        if (!joint_n_) 
        {
          ROS_ERROR("Could not find joint: %s", joint_n_.c_str());
          return false;
        }
      
        link = urdf_model.getParent(link);
      */
        ROS_INFO("NOW ITERATING FROM TIP LINK TO ROOT_LINK");
      }        
    }
   // -----------------------------------------------------------------------
    
    /*
    //Print out the list of all the joints that will be used
    ROS_INFO("liST OF ALL THE JOINTS THAT WILL BE COMMANDED");
    for (unsigned int j=0; j < n_dof_; j++) 
    {
      ROS_INFO("joint  :   %s \n ", joint_n_(j).c_str());
    }
*/
    /*
       const std::vector<std::string>& joint_names = robot -> getNames();

       for (unsigned i=0 ; i < joint_names.size(); i++)
       ROS_DEBUG("Got joint %s", joint_names[i].c_str());

    // get joints handles and joints states
    for (unsigned i=0; i < joint_names.size(); i++)
    {
    joint_handles_[i] = robot -> getHandle(joint_names[i].getName()); 
    }
    */
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
    /*
   //get the positions of all the joints
    for (unsigned i=0; i < num_joints ; i++)
    {
      command_.initRT( joint_handles_[i].getPosition() );
    }
 */
  }
/*
  void InverseDynamicsController:computeInvDyn() 
  {
    // Get the current joint positions and velocities.                                                                                                                              
    for (unsigned int i=0; i < num_joints; i++) 
    {
      q_(i)    = joint_handles_[i].getPosition();
      qdot_(i) = joint_handles_[i].getVelocity();
    }
  
    for (uint i = 0; i < num_joints ; i++) 
    {

      ROS_INFO("TORQUE computation: %lf for joint: %d", torques_(i), i);
     // joint_handles_[i].setCommand( torques_(i));
    }
  
  }
*/
  void InverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
    double command = *(command_.readFromRT());
   /* 
    // Get the current joint positions and velocities.                                                                                                                              
    for (unsigned int i=0; i < num_joints; i++) 
    {
      q_(i)    = joint_handles_[i].getPosition();
      qdot_(i) = joint_handles_[i].getVelocity();
    }
    // Compute inverse dynamics
    // This computes the torques on each joint of the arm as a function of
    // the arm's joint-space position, velocities, accelerations, external
    // forces/torques and gravity.
  
    if(id_solver_ -> CartToJnt(
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
