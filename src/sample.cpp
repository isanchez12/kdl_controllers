









std::vector<uint> InverseDynmaicsController::update(const KDL::JntArray q_current, Eigen::VectorXd& q_reference, Eigen::VectorXd& qdot_reference) 
  {


      computeForwardKinematics(end_effector_pose_right, "/grippoint_right");
          getFKsolution(end_effector_pose_right,"/grippoint_right", poseRight);


  }

void InverseDynamicsController::computeForwardKinematics(KDL::Frame& FK_end_effector_pose,
                                                       const std::string& end_effector_frame) {

  // Compute forward kinematics
  if (end_effector_frame == "/grippoint_right") {
    if (fk_solver_right->JntToCart(q_chain_right_, FK_end_effector_pose) < 0 ) ROS_WARN("Problems with FK computation for the LEFT chain");
  }
     
     
     
     
     
                     
