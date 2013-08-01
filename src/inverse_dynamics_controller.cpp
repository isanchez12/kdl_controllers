
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
   // ,positions_()
     ,q_() //positions
     ,qdot_() //velocities
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
    
  //  gravity_[0]= 0.0;
  //  gravity_[1]= 0.0; 
    gravity_[2]= -9.8;

    // Create inverse dynamics chainsolver
    id_solver_.reset( new KDL::ChainIdSolver_RNE(   
        kdl_chain_,
        KDL::Vector(gravity_[0],gravity_[1],gravity_[2])));


    num_joints = n_dof_;



    // Resize working vectors
    //positions_.resize(n_dof_);   //positions & velocities
    q_.resize(n_dof_);
    qdot_.resize(n_dof_);
    accelerations_.resize(n_dof_);
    torques_.resize(n_dof_);
    ext_wrenches_.resize(kdl_chain_.getNrOfSegments());

    ROS_INFO(" THE SIZE OF KDL_CHAIN from the number fo segments: %d", kdl_chain_.getNrOfSegments());
    ROS_INFO(" SIZE OF ROWS FOR- q_: %d , qdot_: %d, qdotdot_: %d, torques_ : %d",
                q_.rows(), qdot_.rows(), accelerations_.rows(), torques_.rows());
    // Zero out torque data
    torques_.data.setZero();
    accelerations_.data.setZero();

    unsigned int num_actuated_joints_ = 0;

    ROS_INFO("Checking to see which joints are actuated");
    ROS_INFO("Initializing JOINTS FROM THE KDL CHAIN");
    for (size_t i=0; i<kdl_chain_.getNrOfSegments(); i++){

      if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
      { 
        ROS_INFO("Joint '%s' is was found in the kdl chain", kdl_chain_.getSegment(i).getJoint().getName().c_str());
        num_actuated_joints_++;           
      }
    }
    ROS_INFO("NUMBER OF ACTUATED JOINTS : %d", num_actuated_joints_);

      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
    
    //Creating a string vector of all the actuated joint names
    for(std::vector<KDL::Segment>::const_iterator segment=kdl_chain_.segments.begin();
        segment != kdl_chain_.segments.end();
        segment++)
    {   
       joint_names_.push_back(segment->getJoint().getName());
    }
 
    joint_handles_.resize(n_dof_);
  
    for( unsigned int j=0; j < n_dof_; j++)
    {
      ROS_INFO("Joint%d_ '%s' was registerd as a joint name", j, joint_names_[j].c_str());
      //getting joint handle from hardware interfarce
      joint_handles_[j] = robot-> getHandle(joint_names_[j]);
    
    }
    ROS_INFO("N_DOF_ =  '%d' ", n_dof_);
    ROS_INFO("SUCCESSFULLY got the joint handles from the hardware inteface");
    ROS_INFO("INIT SUCCESSFULLY COMPLETED");

    return true;
  }
/*
void InverseDynamicsController::getPositions(std::vector<double> &positions)
{

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
}
  void InverseDynamicsController::update(const ros::Time& time, const ros::Duration& period)
  {
  
    std::vector<double> pos, vel;
    pos.resize(n_dof_);
    vel.resize(n_dof_);
    //GET JOINT POSITIONS AND VELOCITIES
   /////////////////////////////////////////////////////////////////
    for( unsigned int j=0; j < n_dof_; j++) 
    {
         pos[j]  = joint_handles_[j].getPosition();
         vel[j]  = joint_handles_[j].getVelocity();
         //ROS_INFO("POSITION FOR JOINT HANDLE '%d' : '%lf'", j, pos[j]);
         
         //store joint positions in the KDL::JntArray for q_ & qdot_
         q_(j) = pos[j];
         qdot_(j) = vel[j];
        // ROS_INFO("Joint_handle_: %d , POS: %lf, VEL: %lf", j, q_(j), qdot_(j) );
        
    }
     
    /////////////////////////////////////////////////////////////////
    // Compute inverse dynamics
    // This computes the torques on each joint of the arm as a function of
    // the arm's joint-space position, velocities, accelerations, external
    // forces/torques and gravity.
   /* if(id_solver_ -> CartToJnt(
          q_,
          qdot_,
          accelerations_,
          ext_wrenches_,
          torques_) != 0)
    {
      ROS_ERROR("Could not compute joint torques!");
    }
   */ /*
    for (uint i = 0; i < num_joints ; i++) 
    {

      ROS_INFO("TORQUE computation: %lf for joint: %d", torques_(i), i);
     // joint_handles_[i].setCommand( torques_(i));
    }
*/
 //OPTION 2 ------------------------------------------
   
    static unsigned int foo = 0; 

    int id_valid = id_solver_ -> CartToJnt( q_,qdot_,accelerations_,ext_wrenches_, torques_);
   /// Only when a solution is found it will be send
   if ( id_valid >= 0 ) 
   {
     for( unsigned int j=0; j < n_dof_; j++) 
     {
       joint_handles_[j].setCommand( torques_(j));
       
    /*   if(  foo == 10)
       {
         ROS_INFO(" TORQUE OUTPUT for '%d' : %lf ", j, torques_(j));   
         foo = 0;
       }
       */
      }
       if(foo++ % 100 == 0) {
         ROS_DEBUG_STREAM(" TORQUE OUTPUT: " << torques_.data);   
       }
   }
   else
   {
     //  ROS_INFO("ID SOLUTION NOT for inverse dynamics solver!!:\n");
     // DO NOT USE THIS-> ROS_INFO("Positions:%lf  , Velocity: %lf, Torques: %lf ", q_, qdot_, torques_ );
        ROS_INFO(" ID is not Valid and is: %d", id_valid);
   }

  }

  void InverseDynamicsController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    setCommand(msg->data);
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( kdl_controllers::InverseDynamicsController, controller_interface::ControllerBase)
