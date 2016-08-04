#ifndef LWR_HW__LWR_HW_REAL_H
#define LWR_HW__LWR_HW_REAL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// FRIL remote hooks
#include <FastResearchInterface.h>

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK   2000
#define EOK 0

namespace lwr_hw
{

class LWRHWFRIL : public LWRHW
{

public:

  LWRHWFRIL() : LWRHW() {}
  ~LWRHWFRIL() {}

  void stop(){return;};
  void set_mode(){return;};

  void setInitFile(std::string init_file){init_file_ = init_file; file_set_ = true;};

  // Init, read, and write, with FRI hooks
  bool init()
  {
    if( !(file_set_) )
    {
      std::cout << "Did you forget to set the init file?" << std::endl
                << "You must do that before init()" << std::endl
                << "Exiting..." << std::endl;
      return false;
    }

    // construct a low-level lwr
    device_.reset( new FastResearchInterface( init_file_.c_str() ) );

    ResultValue	=	device_->StartRobot( FRI_CONTROL_POSITION );
    if (ResultValue != EOK)
    {
      std::cout << "An error occurred during starting up the robot...\n" << std::endl;
      return false;
    }

    return true;
  }

  void read(ros::Time time, ros::Duration period)
  {
    float msrJntPos[n_joints_];
    float msrJntTrq[n_joints_];

    device_->GetMeasuredJointPositions( msrJntPos );
    device_->GetMeasuredJointTorques( msrJntTrq );

    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = (double)msrJntPos[j];
      joint_position_kdl_(j) = joint_position_[j];
      joint_effort_[j] = (double)msrJntTrq[j];
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.toSec(), joint_velocity_[j], 0.2);
      joint_stiffness_[j] = joint_stiffness_command_[j];
    }
    return;
  }

  void write(ros::Time time, ros::Duration period)
  {
    enforceLimits(period);

    // ensure the robot is powered and it is in control mode, almost like the isMachineOk() of Standford
    if ( device_->IsMachineOK() )
    {
      device_->WaitForKRCTick();

      switch (getControlStrategy())
      {

        case JOINT_POSITION:

          // Ensure the robot is in this mode
          if( (device_->GetCurrentControlScheme() == FRI_CONTROL_POSITION) )
          {
             float newJntPosition[n_joints_];
             for (int j = 0; j < n_joints_; j++)
             {
               newJntPosition[j] = (float)joint_position_command_[j];
             }
             device_->SetCommandedJointPositions(newJntPosition);
          }
          break;

        case CARTESIAN_IMPEDANCE:
          break;

         case JOINT_IMPEDANCE:

          // Ensure the robot is in this mode
          if( (device_->GetCurrentControlScheme() == FRI_CONTROL_JNT_IMP) )
          {
           float newJntPosition[n_joints_];
           float newJntStiff[n_joints_];
           float newJntDamp[n_joints_];
           float newJntAddTorque[n_joints_];

           // WHEN THE URDF MODEL IS PRECISE
           // 1. compute the gracity term
           // f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

           // 2. read gravity term from FRI and add it with opposite sign and add the URDF gravity term
           // newJntAddTorque = gravity_effort_  - device_->getF_DYN??

            for(int j=0; j < n_joints_; j++)
            {
              newJntPosition[j] = (float)joint_position_command_[j];
              newJntAddTorque[j] = (float)joint_effort_command_[j];
              newJntStiff[j] = (float)joint_stiffness_command_[j];
              newJntDamp[j] = (float)joint_damping_command_[j];
            }
            device_->SetCommandedJointStiffness(newJntStiff);
            device_->SetCommandedJointPositions(newJntPosition);
            device_->SetCommandedJointDamping(newJntDamp);
            device_->SetCommandedJointTorques(newJntAddTorque);
          }
          break;

         case GRAVITY_COMPENSATION:
           break;
       }
    }
    return;
  }

  void setControlStrategy(ControlStrategy strategy)
  {
    ResultValue = device_->StopRobot();
    if (ResultValue != EOK)
    {
        std::cout << "An error occurred during stopping the robot, couldn't switch mode...\n" << std::endl;
        return;
    }
    if( strategy == JOINT_POSITION )
    {
      ResultValue = device_->StartRobot( FRI_CONTROL_POSITION );
      if (ResultValue != EOK)
      {
        std::cout << "An error occurred during starting the robot, couldn't switch to JOINT_POSITION...\n" << std::endl;
        return;
      }
    }
    else if( strategy >= JOINT_IMPEDANCE )
    {
      ResultValue = device_->StartRobot( FRI_CONTROL_JNT_IMP );
      if (ResultValue != EOK)
      {
        std::cout << "An error occurred during starting the robot, couldn't switch to JOINT_IMPEDANCE...\n" << std::endl;
        return;
      }
    }
    current_strategy_ = strategy;
  }

private:

  // Parameters
  std::string init_file_;
  bool file_set_ = false;

  // low-level interface
  boost::shared_ptr<FastResearchInterface> device_;
  int ResultValue = 0;
};

}

#endif
