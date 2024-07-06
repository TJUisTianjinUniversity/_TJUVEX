#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftMotorGroupMotorA = motor(PORT19, ratio18_1, false);
motor LeftMotorGroupMotorB = motor(PORT20, ratio18_1, false);
motor_group LeftMotorGroup = motor_group(LeftMotorGroupMotorA, LeftMotorGroupMotorB);
motor RightMotorGroupMotorA = motor(PORT11, ratio18_1, true);
motor RightMotorGroupMotorB = motor(PORT17, ratio18_1, true);
motor_group RightMotorGroup = motor_group(RightMotorGroupMotorA, RightMotorGroupMotorB);
motor MotorarmMotorA = motor(PORT18, ratio36_1, true);
motor MotorarmMotorB = motor(PORT13, ratio36_1, false);
motor_group Motorarm = motor_group(MotorarmMotorA, MotorarmMotorB);
motor Motorhand = motor(PORT16, ratio18_1, false);

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