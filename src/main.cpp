#include "main.h"
#include "display/lv_objx/lv_list.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "pros/vision.h"
#include <string>
#define DEADZONE 15
#define DIGITAL_SENSOR_PORT_A 'A'
#define DIGITAL_SENSOR_PORT_B 'B'
#define DIGITAL_SENSOR_PORT_C 'C'
#define DIGITAL_SENSOR_PORT_D 'D'

// Variables
bool launcher_toggle = false, snarfer_toggle = false, snarfer_reverse_toggle = false, snarf_updated = false, gate_updated = false;
int snarf_dir = 1;
int left_x, left_y, right_x, right_y, snarfer, launcher;
int forward_table[] = {-127,-125,-123,-121,-119,-117,-115,-113,-112,-110,-108,-106,-104,-102,-101,-99,-97,-95,-94,-92,-90,-88,-87,-85,-84,-82,-80,-79,-77,-76,-74,-73,-71,-70,-68,-67,-65,-64,-62,-61,-60,-58,-57,-56,-54,-53,-52,-50,-49,-48,-47,-45,-44,-43,-42,-41,-40,-39,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-26,-25,-24,-23,-22,-21,-20,-20,-19,-18,-17,-17,-16,-15,-15,-14,-13,-13,-12,-11,-11,-10,-10,-9,-9,-8,-8,-7,-7,-6,-6,-5,-5,-5,-4,-4,-3,-3,-3,-3,-2,-2,-2,-2,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,5,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,13,13,14,15,15,16,17,17,18,19,20,20,21,22,23,24,25,26,26,27,28,29,30,31,32,33,34,35,36,37,39,40,41,42,43,44,45,47,48,49,50,52,53,54,56,57,58,60,61,62,64,65,67,68,70,71,73,74,76,77,79,80,82,84,85,87,88,90,92,94,95,97,99,101,102,104,106,108,110,112,113,115,117,119,121,123,125,127};
int turn_table[] = {-63,-62,-61,-60,-59,-58,-57,-56,-55,-54,-53,-53,-52,-51,-50,-49,-48,-47,-46,-46,-45,-44,-43,-42,-41,-41,-40,-39,-38,-38,-37,-36,-35,-35,-34,-33,-32,-32,-31,-30,-30,-29,-28,-28,-27,-26,-26,-25,-24,-24,-23,-23,-22,-21,-21,-20,-20,-19,-19,-18,-18,-17,-17,-16,-16,-15,-15,-14,-14,-13,-13,-12,-12,-11,-11,-11,-10,-10,-9,-9,-9,-8,-8,-8,-7,-7,-7,-6,-6,-6,-5,-5,-5,-5,-4,-4,-4,-4,-3,-3,-3,-3,-2,-2,-2,-2,-2,-2,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,7,7,7,8,8,8,9,9,9,10,10,11,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,23,23,24,24,25,26,26,27,28,28,29,30,30,31,32,32,33,34,35,35,36,37,38,38,39,40,41,41,42,43,44,45,46,46,47,48,49,50,51,52,53,53,54,55,56,57,58,59,60,61,62,63};
int launcher_cycle[] = {87, 112, 127};
int launcher_power = 2; // default power level
bool one_stick = true;
int drive_dir = 1;
bool indexing = false;

// autonomous movement
float Wheel_Diameter = 4;
float Wheel_Circumference = Wheel_Diameter * 3.1416;
float Turning_Diameter = 14.6;
float Turning_Circumference = Turning_Diameter * 3.1416;
float Turning_Distance = 0;
float Wheel_Revolutions = 0;
float Turn_Wheel_Rotation = 0;
float Forward_Wheel_Rotation = 0;
bool wait = false; 

// Defining ports - drive
pros::Motor left_front_motor(19,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_front_motor(11,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_back_motor(20,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_back_motor(12,pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

// other
pros::Motor snarfer_motor(1);
pros::Motor launcher_motor(10,pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor roller_motor(7);
pros::Motor index_motor(2, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);

// pneumatics
pros::ADIDigitalOut gate_raise_pneumatic(DIGITAL_SENSOR_PORT_A);
pros::ADIDigitalOut gate_drop_pneumatic(DIGITAL_SENSOR_PORT_B);
pros::ADIDigitalOut endgame_release(DIGITAL_SENSOR_PORT_C);
pros::ADIDigitalIn indexer_limit_sensor(DIGITAL_SENSOR_PORT_D);

// Set motor groups
pros::Motor_Group left_motors ({left_front_motor, left_back_motor});
pros::Motor_Group right_motors ({right_front_motor, right_back_motor});

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);  
  left_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_front_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_back_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
//float Turning_Power = 127; 
bool check_each_in_vector(float left_goal, float right_goal, float threshold) {
  if ((left_front_motor.get_position() > left_goal+threshold) || left_front_motor.get_position() < left_goal-threshold) {
    return true;
  }
  if ((left_back_motor.get_position() > left_goal+threshold) || left_back_motor.get_position() < left_goal-threshold) {
    return true;
  }
  if ((right_front_motor.get_position() > right_goal+threshold) || right_front_motor.get_position() < right_goal-threshold) {
    return true;
  }
  if ((right_back_motor.get_position() > right_goal+threshold) || right_back_motor.get_position() < right_goal-threshold) {
    return true;
  }
  return false;
}
void turn(float angle, float velocity){
  wait = true; //stop turn from running multiple times
  
  Turning_Distance = angle/360 * Turning_Circumference;
  Wheel_Revolutions = Turning_Distance/Wheel_Circumference;
  Turn_Wheel_Rotation = Wheel_Revolutions*360; // Degrees that the wheel should turn

  left_motors.tare_position();
  right_motors.tare_position();

  left_motors.move_absolute(Turn_Wheel_Rotation, velocity);
  right_motors.move_absolute(Turn_Wheel_Rotation, -velocity);
  while (check_each_in_vector(Turn_Wheel_Rotation, -Turn_Wheel_Rotation, 10)) {
    pros::delay(2);
  }
  left_motors.tare_position();
  right_motors.tare_position();
  
  wait = false; //stop turn from running multiple times
}

void move(float inches, float velocity) {
  wait = true;
  
  Forward_Wheel_Rotation = (inches/Wheel_Circumference)*360;

  left_motors.tare_position();
  right_motors.tare_position();

  left_motors.move_absolute((Forward_Wheel_Rotation),velocity);
  right_motors.move_absolute((Forward_Wheel_Rotation),-velocity);
  while (check_each_in_vector(Forward_Wheel_Rotation, -Forward_Wheel_Rotation, 360)) {
    // Continue running this loop as long as the motors are not within +-5 units of its goal
    pros::lcd::set_text(1,"Running Delay Loop");
    pros::delay(2);
  }
  left_motors.tare_position();
  right_motors.tare_position();

  wait = false;
}
void auton2(){
  roller_motor = 100;
  left_motors.move(60);
  right_motors.move(-60);
  pros::delay(500);
  roller_motor = 0;
  left_motors.move(0);
  right_motors.move(0);

}
void autonomous() {
  auton2();
  // pros::lcd::clear();
  // while (wait) {pros::delay(10);} turn(360, 100);
  // while (wait) {pros::delay(10);} move(48,100);
  // while (wait) {pros::delay(10);} turn(360,100);
  // while (wait) {pros::delay(10);} move(48,100);
  
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off. 
 */

}

// toggles for the pnuematics
// gate state: false = down, true = up
bool gate_state_global = false;
bool gate_state_toggle = true;

void RaiseLowerGate(bool gate_state){
  pros::lcd::print(1, "Raising/Lowering Gate");
  if (gate_state == true){
    gate_drop_pneumatic.set_value(false);
    pros::delay(200);
    gate_raise_pneumatic.set_value(true);
  }
  else{
    gate_raise_pneumatic.set_value(false);
    pros::delay(200);
    gate_drop_pneumatic.set_value(true);
  }
}

// OP CONTROL
void opcontrol() {
  // set braking modes
  index_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  endgame_release.set_value(false);

  // loop!
  while (true) {
    pros::lcd::clear();

    // Get joystick values
    left_y = -1*(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    left_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
    right_y = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    right_x = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

    // gate drop
    if (snarf_updated){
      gate_state_global = !snarfer_toggle;
      gate_state_toggle = true;
      snarf_updated = false;
    }

    if(gate_state_toggle){
      if (gate_state_global){
        pros::lcd::set_text(2, "Gate Up");
      }
      if (!gate_state_global){
        pros::lcd::set_text(2, "Gate Down");
      }
      RaiseLowerGate(gate_state_global);
      gate_state_toggle = false;
      gate_updated = true;
    }

    // display launcher motor power level on controller (not persistent)
    controller.print(0,4,"Power: %d ", launcher_cycle[launcher_power]);

    // toggle launcher power modes(using the up and down arrows on dpad)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && launcher_power < 2) {
      launcher_power += 1;
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) && launcher_power > 0) {
      launcher_power -= 1;
    }

    
    // Toggle launcher (R1)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) == 1) {
      launcher_toggle = !launcher_toggle;
    }
    if (launcher_toggle) {
      launcher_motor.move(launcher_cycle[launcher_power]);
    } else {
      launcher_motor.move(0);
    }

    // Indexer (R2)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
      if (!indexing){
        if(indexer_limit_sensor.get_value()){
          index_motor.move_relative(90,127);
        }
        else{
          indexing=true;
          index_motor.move_relative(10, 127);
        }
      }
    }
    if (indexing){
      if (indexer_limit_sensor.get_value()){
        indexing=false;
      }
      else{
        index_motor.move_relative(20,127);
      }
    }
    
    // Toggle snarfer (L1)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) == 1) {
      snarfer_toggle = !snarfer_toggle;
      snarf_updated = true;
    }
    // update our gate and snarfer states
    if (gate_updated){
      pros::delay(500);
      gate_updated = false;
      if (snarfer_toggle) {
        snarfer_motor = 127*snarf_dir;
        drive_dir = 1;
      } else {
        snarfer_motor = 0;
        drive_dir = -1;
      }
    }

    // ROLLER (A/X)
    // enclosing if to make sure the motor speeds are not being set rapidly
    if (!((controller.get_digital(DIGITAL_A)) && (controller.get_digital(DIGITAL_X)))) {
      if (controller.get_digital(DIGITAL_A)) {
        roller_motor = -100;
      } else if (controller.get_digital(DIGITAL_X)) {
        roller_motor = 100;
      } else {
        roller_motor = 0;
      }
    }

    // Endgame
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
      if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        // fire endgame!
        endgame_release.set_value(true);
      }
    }

    // Drive Control Loop (LEFT)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      one_stick = !one_stick;
    }
    if (one_stick){
      if (drive_dir == 1){ // when we are shooting
        left_motors.move(forward_table[left_x+127] + forward_table[left_y+127]);
        right_motors.move(forward_table[left_x+127] - forward_table[left_y+127]);
      }
      else{ // reversed cotrols when snarfing so the snarf = front
        left_motors.move((forward_table[left_x+127] - forward_table[left_y+127]));
        right_motors.move((forward_table[left_x+127] + forward_table[left_y+127]));
      }
    } else { // non one stick, then tank drive!
      right_motors.move(-1*forward_table[left_y+127]);
      left_motors.move(-1*forward_table[right_y+127]);
    }

    pros::delay(20);
  }
}