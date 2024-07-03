#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftMotorGroupMotorA = motor(PORT1, ratio18_1, false);
motor LeftMotorGroupMotorB = motor(PORT2, ratio18_1, false);
motor_group LeftMotorGroup = motor_group(LeftMotorGroupMotorA, LeftMotorGroupMotorB);
motor RightMotorGroupMotorA = motor(PORT3, ratio18_1, true);
motor RightMotorGroupMotorB = motor(PORT4, ratio18_1, true);
motor_group RightMotorGroup = motor_group(RightMotorGroupMotorA, RightMotorGroupMotorB);
motor MotorarmMotorA = motor(PORT5, ratio36_1, true);
motor MotorarmMotorB = motor(PORT6, ratio36_1, false);
motor_group Motorarm = motor_group(MotorarmMotorA, MotorarmMotorB);
motor Motorhand = motor(PORT11, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}