#ifndef LWR_HW__IMPEDANCE_INTERFACE_H
#define LWR_HW__IMPEDANCE_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>

namespace hardware_interface
{

/** \brief Hardware interface to support configuring an array of joints.
 *
 * This \ref HardwareInterface supports configuring the output of an array of
 * named joints. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double. To specify a meaning to 
 * this command, see the derived classes like \ref StiffnessJointInterface etc.
 *
 * \note Getting a joint handle through the getHandle() method \e will not
 * claim that resource.
 *
 */
class JointSettingsInterface : public HardwareResourceManager<JointHandle, DontClaimResources> {};

/// \ref JointCommandInterface for commanding stiffness-based joints.
class StiffnessJointInterface : public JointSettingsInterface {};

/// \ref JointCommandInterface for commanding damping-based joints.
class DampingJointInterface : public JointSettingsInterface {};

}

#endif
