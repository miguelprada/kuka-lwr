#include "lwr_hw/lwr_hw.h"

#include <set>

namespace lwr_hw
{
  void LWRHW::create(std::string name, std::string urdf_string)
  {
    std::cout << "Creating a KUKA LWR 4+ called: " << name << std::endl;

    // SET NAME AND MODEL
    robot_namespace_ = name;
    urdf_string_ = urdf_string;

    // ALLOCATE MEMORY

    // JOINT NAMES ARE TAKEN FROM URDF NAME CONVENTION
    joint_names_.push_back( robot_namespace_ + std::string("_0_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_1_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_2_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_3_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_4_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_5_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_6_joint") );

    // VARIABLES
    joint_position_.resize(n_joints_);
    joint_position_prev_.resize(n_joints_);
    joint_velocity_.resize(n_joints_);
    joint_effort_.resize(n_joints_);
    joint_stiffness_.resize(n_joints_);
    joint_damping_.resize(n_joints_);
    joint_position_command_.resize(n_joints_);
    joint_velocity_command_.resize(n_joints_);
    joint_effort_command_.resize(n_joints_);
    joint_stiffness_command_.resize(n_joints_);
    joint_damping_command_.resize(n_joints_);

    joint_lower_limits_.resize(n_joints_);
    joint_upper_limits_.resize(n_joints_);
    joint_lower_limits_stiffness_.resize(n_joints_);
    joint_upper_limits_stiffness_.resize(n_joints_);
    joint_lower_limits_damping_.resize(n_joints_);
    joint_upper_limits_damping_.resize(n_joints_);
    joint_effort_limits_.resize(n_joints_);

    // RESET VARIABLES
    reset();

    std::cout << "Parsing transmissions from the URDF..." << std::endl;

    // GET TRANSMISSIONS THAT BELONG TO THIS LWR 4+ ARM
    if (!parseTransmissionsFromURDF(urdf_string_))
    {
      std::cout << "lwr_hw: " << "Error parsing URDF in lwr_hw.\n" << std::endl;
      return;
    }

    std::cout << "Registering interfaces..." << std::endl;

    const urdf::Model *const urdf_model_ptr = urdf_model_.initString(urdf_string_) ? &urdf_model_ : NULL;
    registerInterfaces(urdf_model_ptr, transmissions_);

    std::cout << "Initializing KDL variables..." << std::endl;

    // INIT KDL STUFF
    initKDLdescription(urdf_model_ptr);

    std::cout << "Succesfully created an abstract LWR 4+ ARM with interfaces to ROS control" << std::endl;
  }

  // reset values
  void LWRHW::reset()
  {
    for (int j = 0; j < n_joints_; ++j)
    {
      joint_position_[j] = 0.0;
      joint_position_prev_[j] = 0.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 0.0;
      joint_stiffness_[j] = 0.0;
      joint_damping_[j] = 0.0;

      joint_position_command_[j] = 0.0;
      joint_velocity_command_[j] = 0.0;
      joint_effort_command_[j] = 0.0;
      joint_stiffness_command_[j] = 2000.0;
      joint_damping_command_[j] = 0.7;
    }

    current_strategy_ = JOINT_POSITION;

    return;
  }

  void LWRHW::registerInterfaces(const urdf::Model *const urdf_model, 
                     std::vector<transmission_interface::TransmissionInfo> transmissions)
  {

    // Check that this transmission has one joint
    if( transmissions.empty() )
    {
      std::cout << "lwr_hw: " << "There are no transmission in this robot, all are non-driven joints? " 
        << std::endl;
      return;
    }

    // Initialize values
    for(int j=0; j < n_joints_; j++)
    {
      // Check that this transmission has one joint
      if(transmissions[j].joints_.size() == 0)
      {
        std::cout << "lwr_hw: " << "Transmission " << transmissions[j].name_
          << " has no associated joints." << std::endl;
        continue;
      }
      else if(transmissions[j].joints_.size() > 1)
      {
        std::cout << "lwr_hw: " << "Transmission " << transmissions[j].name_
          << " has more than one joint, and they can't be controlled simultaneously"
          << std::endl;
        continue;
      }

      std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;

      if( joint_interfaces.empty() )
      {
        std::cout << "lwr_hw: " << "Joint " << transmissions[j].joints_[0].name_ <<
          " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
          "You need to, otherwise the joint can't be controlled." << std::endl;
        continue;
      }

      const std::string& hardware_interface = joint_interfaces.front();

      // Debug
      std::cout << "\x1B[37m" << "lwr_hw: " << "Loading joint '" << joint_names_[j]
        << "' of type '" << hardware_interface << "'" << "\x1B[0m" << std::endl;

      // Create joint state interface for all joints
      state_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

      // Decide what kind of command interface this actuator/joint has
      hardware_interface::JointHandle joint_handle_effort;
      joint_handle_effort = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
                                                       &joint_effort_command_[j]);
      effort_interface_.registerHandle(joint_handle_effort);

      hardware_interface::JointHandle joint_handle_position;
      joint_handle_position = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
                                                       &joint_position_command_[j]);
      position_interface_.registerHandle(joint_handle_position);

      hardware_interface::JointHandle joint_handle_stiffness;
      joint_handle_stiffness = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
                                                       &joint_stiffness_command_[j]);
      stiffness_interface_.registerHandle(joint_handle_stiffness);

      hardware_interface::JointHandle joint_handle_damping;
      joint_handle_damping = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
                                                       &joint_damping_command_[j]);
      damping_interface_.registerHandle(joint_handle_damping);

     // velocity command handle, recall it is fake, there is no actual velocity interface
      hardware_interface::JointHandle joint_handle_velocity;
      joint_handle_velocity = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
          &joint_velocity_command_[j]);

      registerJointLimits(joint_names_[j], 
                          joint_handle_effort, 
                          joint_handle_position,
                          joint_handle_velocity,
                          joint_handle_stiffness,
                          joint_handle_damping,
                          urdf_model, 
                          &joint_lower_limits_[j], &joint_upper_limits_[j],
                          &joint_lower_limits_stiffness_[j],
                          &joint_upper_limits_stiffness_[j],
                          &joint_lower_limits_damping_[j],
                          &joint_upper_limits_damping_[j],
                          &joint_effort_limits_[j]);
    }

    // Register interfaces
    registerInterface(&state_interface_);
    registerInterface(&effort_interface_);
    registerInterface(&position_interface_);
    registerInterface(&stiffness_interface_);
    registerInterface(&damping_interface_);
  }

  // Register the limits of the joint specified by joint_name and\ joint_handle. The limits are
  // retrieved from the urdf_model.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void LWRHW::registerJointLimits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle_effort,
                           const hardware_interface::JointHandle& joint_handle_position,
                           const hardware_interface::JointHandle& joint_handle_velocity,
                           const hardware_interface::JointHandle& joint_handle_stiffness,
                           const hardware_interface::JointHandle& joint_handle_damping,
                           const urdf::Model *const urdf_model,
                           double *const lower_limit, double *const upper_limit, 
                           double *const lower_limit_stiffness, double *const upper_limit_stiffness,
                           double *const lower_limit_damping, double *const upper_limit_damping,
                           double *const effort_limit)
  {
    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();
    *lower_limit_stiffness = -std::numeric_limits<double>::max();
    *upper_limit_stiffness = std::numeric_limits<double>::max();
    *lower_limit_damping= -std::numeric_limits<double>::max();
    *upper_limit_damping= std::numeric_limits<double>::max();
    *effort_limit = std::numeric_limits<double>::max();

    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    if (urdf_model != NULL)
    {
      const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
      if (urdf_joint != NULL)
      {
        // Get limits from the URDF file.
        if (joint_limits_interface::getJointLimits(urdf_joint, limits))
          has_limits = true;
        if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
          has_soft_limits = true;
      }
    }

    if (!has_limits)
      return;

    if (limits.has_position_limits)
    {
      *lower_limit = limits.min_position;
      *upper_limit = limits.max_position;
    }
    if (limits.has_effort_limits)
      *effort_limit = limits.max_effort;

    if (has_soft_limits)
    {
      const joint_limits_interface::EffortJointSoftLimitsHandle limits_handle_effort(joint_handle_effort, limits, soft_limits);
      ej_limits_interface_.registerHandle(limits_handle_effort);
      const joint_limits_interface::PositionJointSoftLimitsHandle limits_handle_position(joint_handle_position, limits, soft_limits);
      pj_limits_interface_.registerHandle(limits_handle_position);
      const joint_limits_interface::VelocityJointSoftLimitsHandle limits_handle_velocity(joint_handle_velocity, limits, soft_limits);
      vj_limits_interface_.registerHandle(limits_handle_velocity);

    }
    else
    {
      const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, limits);
      ej_sat_interface_.registerHandle(sat_handle_effort);
      const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, limits);
      pj_sat_interface_.registerHandle(sat_handle_position);
      const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, limits);
      vj_sat_interface_.registerHandle(sat_handle_velocity);
    }

    *lower_limit_stiffness = 0.0;
    *upper_limit_stiffness = 2000.0;
    *lower_limit_damping = 0.0;
    *upper_limit_damping = 1.0;

    joint_limits_interface::JointLimits stiffness_limits;
    stiffness_limits.has_position_limits = true;
    stiffness_limits.min_position = *lower_limit_stiffness;
    stiffness_limits.max_position = *upper_limit_stiffness;

    const joint_limits_interface::PositionJointSaturationHandle sat_handle_stiffness(joint_handle_stiffness, stiffness_limits);
    sj_sat_interface_.registerHandle(sat_handle_stiffness);

    joint_limits_interface::JointLimits damping_limits;
    damping_limits.has_position_limits = true;
    damping_limits.min_position = *lower_limit_damping;
    damping_limits.max_position = *upper_limit_damping;

    const joint_limits_interface::PositionJointSaturationHandle sat_handle_damping(joint_handle_damping, damping_limits);
    dj_sat_interface_.registerHandle(sat_handle_damping);
  }

  void LWRHW::enforceLimits(ros::Duration period)
  {
    ej_sat_interface_.enforceLimits(period);
    ej_limits_interface_.enforceLimits(period);
    vj_sat_interface_.enforceLimits(period);
    vj_limits_interface_.enforceLimits(period);
    pj_sat_interface_.enforceLimits(period);
    pj_limits_interface_.enforceLimits(period);
    sj_sat_interface_.enforceLimits(period);
    dj_sat_interface_.enforceLimits(period);
  }

  // Get Transmissions from the URDF
  bool LWRHW::parseTransmissionsFromURDF(const std::string& urdf_string)
  {
    std::vector<transmission_interface::TransmissionInfo> transmissions;

    // Only *standard* transmission_interface are parsed
    transmission_interface::TransmissionParser::parse(urdf_string, transmissions);

    // Now iterate and save only transmission from this robot
    for (int j = 0; j < n_joints_; ++j)
    {
      // std::cout << "Check joint " << joint_names_[j] << std::endl;
      std::vector<transmission_interface::TransmissionInfo>::iterator it = transmissions.begin();
      for(; it != transmissions.end(); ++it)
      {
        // std::cout << "With transmission " << it->name_ << std::endl;
        if (joint_names_[j].compare(it->joints_[0].name_) == 0)
        {
          transmissions_.push_back( *it );
          // std::cout << "Found a match for transmission " << it->name_ << std::endl;
        }
      }
    }

    if( transmissions_.empty() )
      return false;

    return true;
  }

  // Init KDL stuff
  bool LWRHW::initKDLdescription(const urdf::Model *const urdf_model)
  {
    // KDL code to compute f_dyn(q)
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    std::cout << "LWR kinematic successfully parsed with " 
              << kdl_tree.getNrOfJoints() 
              << " joints, and " 
              << kdl_tree.getNrOfJoints() 
              << " segments." << std::endl;

    // Get the info from parameters
    std::string root_name;
    ros::param::get(std::string("/") + robot_namespace_ + std::string("/root"), root_name);
    if( root_name.empty() )
      root_name = kdl_tree.getRootSegment()->first; // default
    
    std::string tip_name;
    ros::param::get(std::string("/") + robot_namespace_ + std::string("/tip"), tip_name);
    if( root_name.empty() )
      tip_name = robot_namespace_ + std::string("_7_link"); ; // default

    std::cout << "Using root: " << root_name << " and tip: " << tip_name << std::endl;

    // this depends on how the world frame is set, in all our setups, world has always positive z pointing up.
    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    // Extract the chain from the tree
    if(!kdl_tree.getChain(root_name, tip_name, lwr_chain_))
    {
        ROS_ERROR("Failed to get KDL chain from tree: ");
        return false;
    }

    ROS_INFO("Number of segments: %d", lwr_chain_.getNrOfSegments());
    ROS_INFO("Number of joints in chain: %d", lwr_chain_.getNrOfJoints());

    f_dyn_solver_.reset(new KDL::ChainDynParam(lwr_chain_,gravity_));

    joint_position_kdl_ = KDL::JntArray(lwr_chain_.getNrOfJoints());
    gravity_effort_ = KDL::JntArray(lwr_chain_.getNrOfJoints());

    return true;
  }

  bool LWRHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
  {
    std::set<ControlStrategy> desired_strategies;
    
    for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it )
    {
      for( std::vector<hardware_interface::InterfaceResources>::const_iterator res_it = it->claimed_resources.begin(); res_it != it->claimed_resources.end(); ++res_it )
      {
        // If any of the controllers in the start list works on a velocity interface, the switch can't be done.
        if( res_it->hardware_interface.compare( std::string("hardware_interface::VelocityJointInterface") ) == 0 )
        {
          std::cout << "The given controllers to start work on a velocity joint interface, and this robot does not have such an interface."
                    << "The switch can't be done" << std::endl;
          return false;
        }

        if( res_it->hardware_interface.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
        {
          // Debug
          // std::cout << "One controller wants to work on hardware_interface::PositionJointInterface" << std::endl;
          desired_strategies.insert( JOINT_POSITION );
        }
        else if( res_it->hardware_interface.compare( std::string("hardware_interface::EffortJointInterface") ) == 0 )
        {
          // Debug
          // std::cout << "One controller wants to work on hardware_interface::EffortJointInterface" << std::endl;
          desired_strategies.insert( JOINT_EFFORT );
        }
        else if( res_it->hardware_interface.compare( std::string("hardware_interface::StiffnessJointInterface") ) == 0 )
        {
          // Debug
          // std::cout << "One controller wants to work on hardware_interface::StiffnessJointInterface" << std::endl;
          desired_strategies.insert( JOINT_IMPEDANCE );
        }
        else if( res_it->hardware_interface.compare( std::string("hardware_interface::DampingJointInterface") ) == 0 )
        {
          // Debug
          // std::cout << "One controller wants to work on hardware_interface::DampingJointInterface" << std::endl;
          desired_strategies.insert( JOINT_IMPEDANCE );
        }
        else
        {
          // Debug
          // std::cout << "This controller does not use any command interface, so it is only sensing, no problem" << std::endl;
        }
      }
    }

    ControlStrategy desired_strategy;

    if( desired_strategies.size() == 1 )
    {
      desired_strategy = *(desired_strategies.begin());
      if( desired_strategy == JOINT_IMPEDANCE )
      {
        ROS_ERROR( "Controlling the joint impedance without controlling the target position is not supported." );
        return false;
      }
    }
    else if( desired_strategies.size() > 1 )
    {
      if( desired_strategies.find( JOINT_EFFORT ) != desired_strategies.end() )
      {
        ROS_ERROR( "Effort interface can not be combined with position, stiffness or damping." );
        return false;
      }

      // else desired strategies = {JOINT_POSITION,JOINT_IMPEDANCE}
      desired_strategy = JOINT_IMPEDANCE;
    }
    else // desired_strategies.size() == 0
    {
      ROS_WARN( "No interface required by controller. Hope nothing bad happens..." );
      desired_strategy = getControlStrategy();
    }

    next_strategy_ = desired_strategy;

    return true;
  }

  void LWRHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
  {
    for (int j = 0; j < n_joints_; ++j)
    {
      ///semantic Zero
      joint_position_command_[j] = joint_position_[j];
      joint_effort_command_[j] = 0.0;

      ///call setCommand once so that the JointLimitsInterface receive the correct value on their getCommand()!
      try{  position_interface_.getHandle(joint_names_[j]).setCommand(joint_position_command_[j]);  }
      catch(const hardware_interface::HardwareInterfaceException&){}
      try{  effort_interface_.getHandle(joint_names_[j]).setCommand(joint_effort_command_[j]);  }
      catch(const hardware_interface::HardwareInterfaceException&){}

      ///reset joint_limit_interfaces
      pj_sat_interface_.reset();
      pj_limits_interface_.reset();
      sj_sat_interface_.reset();
      dj_sat_interface_.reset();
    }

    if(next_strategy_ == getControlStrategy())
    {
      std::cout << "The ControlStrategy didn't changed, it is already: " << getControlStrategy() << std::endl;
    }
    else
    {
      setControlStrategy(next_strategy_);
      std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
    }
  }

  
  
}
