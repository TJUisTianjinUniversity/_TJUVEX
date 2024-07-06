using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor_group LeftMotorGroup;
extern motor_group RightMotorGroup;
extern motor_group Motorarm;
extern motor Motorhand;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );