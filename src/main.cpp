#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "robodash/api.h"

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(16);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-15);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 1.5);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -1.5);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Inertial Sensor on port 17
pros::Imu imu(6);

// motor groups
pros::MotorGroup leftMotors({-10, 2, 9},pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({8, -1, -7}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11, // 11 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 600
                              4 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(6.3, // proportional gain (kP)//
                                            0, // integral gain (kI) 
                                            25, // derivative gain (kD) //
                                            0, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            4, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2.8, // proportional gain (kP) //2.8
                                             0, // integral gain (kI)
                                             25.5, // derivative gain (kD) //25.5
                                             0, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             5, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// controller


//Pneumatics (pros::ADIDigitalOut _NAME_ (ADI_PORT))
pros::ADIDigitalOut clamp('F');
pros::ADIDigitalOut doinker('G');
pros::ADIDigitalOut hang('H');


pros::ADIAnalogIn line_tracker ('A');

//Other motors
pros::Motor intake1(-11, pros::MotorGearset::blue);
pros::Motor intake2(-20, pros::MotorGearset::green);

pros::Motor lb1 (21, pros::MotorGearset::green);

//rotation sensor
pros::Rotation ladyBrownRotation(17); //make sure you get the right port
pros::Distance clamp_sensor(5);
pros::Distance reset_distance(14);

//optical
pros::Optical optical(18);

//current team color
char team_color = 'B';

bool intake_on = false;
bool reversed = false;

bool auton = false;
bool opC = false;

float targetTheta = 0;
float exitRange = 0;

bool clampOn = false;
bool current = false;

bool ringStop = false;
bool is_ring_stopped = false;

int sequenceStep = 0;

bool is_stuck = false;

bool skills_hold = false;
bool is_ring_stopped_s = false;
bool first_stage = false;

void clampCheck(void* param){
    bool check = false;
    int distance;
    

    while(true){
        distance = clamp_sensor.get_distance();
        //pros::lcd::print(6, "%i", distance);
        if (clampOn && distance < 25 && !current){
            pros::delay(250);
            clamp.set_value(true);
            current = true;
        }
        else if (!clampOn && current){
            clamp.set_value(false);
            current = !current;
            pros::delay(100);
        }
        
        int dynamic_delay = (clampOn) ? 10 : 100;
        pros::delay(dynamic_delay);
        if (opC) break;
    }
}


void color_check(void* param) {
    // Variables for RGB values
    float hue = 0;
    int x = 0;
    
    skills_hold = false;
    first_stage = false;
    ringStop = false;

    while (true) {
        // Read RGB values from the optical sensor
        auto k = optical.get_proximity();
        auto c = optical.get_rgb();
        
        if (!is_stuck && !first_stage){
            is_ring_stopped = false;
            if(ringStop){
                if ((k > 60 && c.brightness > 0.05) && (c.red > c.blue) == (team_color == 'R') || x > 300){
                    intake1.move(0);
                    intake2.move(0);
                    while (ringStop){
                        is_ring_stopped = true;
                        pros::delay(200);
                    }
                }
                else{
                    x++;
                    intake1.move(-100);
                    intake2.move(127);
                }
                
            }
            else if ((k > 100 && c.brightness > 0.05) && (c.red > c.blue) == (team_color == 'B')) { // ((team_color == 'R' && hue > 137) || (team_color == 'B' && hue < 22))
                // Wrong color detected: Reject object
                pros::delay(150);
                intake1.move(120); 
                pros::delay(200);
            } 
            
            else {
                // Correct color detected or no wrong color
                if (intake_on) {
                    intake1.move(-127); // Keep the intake running forward
                    intake2.move(127);
                } else if (!reversed){
                    intake1.move(0);   // Stop the intake if not needed
                    intake2.move(0);
                }
            }
        }

        // Small delay to avoid overwhelming the system
        int dynamic_delay = (intake_on) ? 10 : 50;
        pros::delay(dynamic_delay);
        if (opC) break;
    }
}

lemlib::PID armPID(1.6, 0, 0);

void LBpidTask(void* param) {
        float currentTheta = 0;
        float prevMotorPower = 0;
        exitRange = 40;
        float error = 0.0f;
        bool allianceS = false;
        int sequenceStepA = 0;
        bool deScore = false;
        skills_hold = false;
        pros::delay(25);

        while(true){

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {

                allianceS = false;
                sequenceStep = (sequenceStep + 1) % 3;

                switch (sequenceStep) {
                    case 1:
                        targetTheta = 26;
                        lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        exitRange = 0.2;
                        break;
                    case 9:
                        targetTheta = 100;
                        //exitRange = 0.2;
                        break;
                    case 2:
                        targetTheta = 210;
                        lb1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        exitRange = 40;
                        break;
                }
            }

            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
                
                switch (sequenceStepA) {
                        case 0:
                            targetTheta = 250;
                            lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                            exitRange = 10;
                            break;
                    
                     case 1:
                        targetTheta = 0;
                        lb1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        exitRange = 10;
                        
                        break;
                }
                sequenceStepA = (sequenceStepA + 1) % 2;
                    allianceS = !allianceS;
            }

            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
                    if (deScore == false){
                        targetTheta = 180;
                        lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        exitRange = 2;
                    }
                    else{
                        sequenceStep = 0;
                        targetTheta = 0;
                        lb1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        exitRange = 10;
                    }
                    deScore = !deScore;
            }

            


                currentTheta = (float)ladyBrownRotation.get_angle() / 100;

                if (currentTheta > 350){
                    currentTheta = currentTheta - 360;
                }
                // Get output from PID (Target - Actual (Accounts for gear ratio))
                

                error = targetTheta - currentTheta;
                
                if (fabs(error) > exitRange) {
                    double out = armPID.update(error);
                    
                    lb1.move_voltage(out * 100);  // Output to motor
                } 
                else if (sequenceStep == 2 && fabs(error) < 30){
                        sequenceStep = 0;
                        targetTheta = 0;
                        exitRange = 40;
                }
                else lb1.brake();  // Stop the motor when within range with said brake mode
                
                pros::delay(20);  // Don't hog the CPU
            
        } 
}

void hang_task(void* param){
    while(true){
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                if (sequenceStep == 1){
                    targetTheta = 0;
                    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    exitRange = 1;
                    pros::delay(550);
                    hang.set_value(true);
                    pros::delay(200);
                    targetTheta = 41;
                    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    exitRange = 1;
                }
                else{
                    hang.set_value(true);
                    pros::delay(200);
                    targetTheta = 41;
                    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    exitRange = 1;
                }
                    
        }
    }
    pros::delay(30);
}


void is_stuck_check(void* param) {
    while (!opC) {
        float intake_torque = intake1.get_torque();  // Store once
        float intake_velocity = intake1.get_actual_velocity();  

        if (intake_torque > 0.34 && fabs(intake_velocity) < 1 && !reversed) {
            is_stuck = true;
            intake1.move(90);
            pros::delay(100);
            intake1.move(0);
        } 
        else {
            is_stuck = false;
            int dynamic_delay = (intake_on) ? 10 : 150;
            pros::delay(dynamic_delay);
        }
    }
}
bool line_detect = false;
bool track = false;

int temp = 0;

void red_SAWP() {
    team_color = 'R';
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 5, 1000);
    targetTheta = 220;
    exitRange = 5;
    pros::delay(350);
    chassis.moveToPoint(0, -3, 1000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 3});
    //chassis.moveToPoint(27.018, 11.853, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 10});
    chassis.moveToPoint(-16.7, -32.9, 2000, {.forwards = false, .maxSpeed = 110});
    chassis.waitUntil(9);
    chassis.cancelMotion();
    chassis.moveToPoint(-16.7, -32.9, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 30});
    clampOn = true;
    targetTheta = 0;
    exitRange = 30;
    while(!current){
        pros::delay(10);
    }
    pros::delay(50);
    chassis.cancelMotion();
    chassis.turnToPoint(-4.26, -42.55, 690, {.minSpeed = 50, .earlyExitRange = 3});
    chassis.waitUntilDone();
    //intake1.move(-127);
    //intake2.move(127);
    intake_on = true;
    chassis.moveToPoint(-4.26, -42.55, 1000, {.minSpeed = 40, .earlyExitRange = 3});
    chassis.moveToPoint(-5, -41, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 3});
    chassis.swingToHeading(30, DriveSide::LEFT, 1000, {.minSpeed = 70, .earlyExitRange = 8});
    //chassis.moveToPose(10.5, -34.5, 0, 1000, {.minSpeed = 60, .earlyExitRange = 5});
    chassis.moveToPose(2.15, -13, -30, 1000, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPoint(-30, 18.27, 2000, {.maxSpeed = 80});
    chassis.waitUntil(13);
    clampOn = false;
    //intake1.move(0);
    //intake2.move(0);
    ringStop = true;
    
    
    
    chassis.swingToPoint(-47.77, 6.67, DriveSide::LEFT ,1000, {.forwards = false, .minSpeed = 30 ,.earlyExitRange = 8});
    chassis.waitUntilDone();
    clampOn = true;
    first_stage = true;
    intake2.move(127);
    chassis.moveToPoint(-57.76, 6.5, 1000, {.forwards = false, .maxSpeed = 40});
    while(!current){
        pros::delay(10);
    }
    chassis.cancelMotion();
    pros::delay(90);
    first_stage = false;
    ringStop = false;
    chassis.swingToPoint(-67.13, 20.95, DriveSide::LEFT, 1000, {.minSpeed = 50, .earlyExitRange = 9});
    chassis.waitUntilDone();
    //intake1.move(-127);
    //intake2.move(127);
    intake_on = true;
    chassis.moveToPoint(-67.75, 24.2, 1000); // test
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.swingToPoint(-60.3, 5.37, DriveSide::RIGHT, 1000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 40, .earlyExitRange = 9, });
    chassis.moveToPoint(-60.3, 5.37, 1000, {.earlyExitRange = 4});
    chassis.waitUntil(5);
    targetTheta = 210;
    exitRange = 90;
}
void blue_SAWP() {
    team_color = 'B';
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 5, 1000);
    targetTheta = 220;
    exitRange = 5;
    pros::delay(350);
    chassis.moveToPoint(0, -3, 1000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 3});
    //chassis.moveToPoint(27.018, 11.853, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 10});
    chassis.moveToPoint(16.7, -32.9, 2000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntil(9);
    chassis.cancelMotion();
    chassis.moveToPoint(16.7, -32.9, 2000, {.forwards = false, .maxSpeed = 40, .minSpeed = 5});
    clampOn = true;
    while(!current){
        pros::delay(10);
    }
    targetTheta = 0;
    exitRange = 30;
    pros::delay(120);
    chassis.cancelMotion();
    chassis.turnToPoint(4.26, -42.55, 690, {.minSpeed = 50, .earlyExitRange = 3});
    
    chassis.waitUntilDone();
    //intake1.move(-127);
    //intake2.move(127);
    intake_on = true;
    chassis.moveToPoint(4.26, -42.55, 1000, {.minSpeed = 40, .earlyExitRange = 3});
    chassis.swingToHeading(-32, DriveSide::RIGHT, 1000, {.minSpeed = 70, .earlyExitRange = 8});
    chassis.moveToPose(-0.4, -11.84, 45.24, 1000, {.minSpeed = 70, .earlyExitRange = 8});
    chassis.moveToPoint(29.6, 18, 2000, {.maxSpeed = 80});
    chassis.waitUntil(8);
    clampOn = false;
    ringStop = true;
    while (!is_ring_stopped){
        pros::delay(10);
    }
    first_stage = true;
    intake2.move(127);
    chassis.swingToPoint(56.355, 7.8, DriveSide::RIGHT ,1000, {.forwards = false, .minSpeed = 30 , .earlyExitRange = 8});

    chassis.waitUntilDone();
    clampOn = true;
    chassis.moveToPoint(56.355, 7.8, 1000, {.forwards = false, .maxSpeed = 40});
    while(!current){
        pros::delay(10);
    }
    chassis.cancelMotion();
    pros::delay(100);
    first_stage = false;
    ringStop = false;
    chassis.swingToPoint(72.1, 20.35, DriveSide::RIGHT, 1000, {.minSpeed = 50, .earlyExitRange = 9});
    chassis.waitUntilDone();
    //intake1.move(-127);
    //intake2.move(127);
    intake_on = true;
    chassis.moveToPoint(72.1, 20.35, 1000);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.swingToPoint(60.3, 5.37, DriveSide::LEFT, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 40, .earlyExitRange = 9});
    chassis.moveToPoint(60.3, 5.37, 1000, {.earlyExitRange = 4});
    chassis.waitUntil(6);
    targetTheta = 210;
    exitRange = 90;
}
void red_ring() {
    team_color = 'R';
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 5, 1000);
    targetTheta = 220;
    exitRange = 5;
    pros::delay(350);
    chassis.moveToPoint(0, -3, 1000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 3});
    //chassis.moveToPoint(27.018, 11.853, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 10});
    chassis.moveToPoint(-16.7, -32.9, 2000, {.forwards = false, .maxSpeed = 110});
    chassis.waitUntil(9);
    chassis.cancelMotion();
    chassis.moveToPoint(-16.7, -32.9, 2000, {.forwards = false, .maxSpeed = 50, .minSpeed = 30});
    clampOn = true;
    while(!current){
        pros::delay(10);
    }
    pros::delay(50);
    targetTheta = 0;
    exitRange = 30;
    chassis.cancelMotion();
    chassis.turnToHeading(195, 1000);
    
    chassis.moveToPose(-21.3, -49.33, 145, 900, {.lead = 0.2, .maxSpeed = 50, .minSpeed = 32, .earlyExitRange = 9});
    chassis.swingToHeading(135, DriveSide::LEFT, 1000, {.minSpeed = 48, .earlyExitRange = 6});
    chassis.waitUntilDone();
    intake_on = true;
    chassis.moveToPoint(-10.5, -61.55, 1000);
    chassis.waitUntilDone();
    pros::delay(700);
    chassis.moveToPose(-18.66, -28, 2000, 1000, {.forwards = false, .lead = 0.3});
    chassis.turnToPoint(-3.1, -45.6, 800, {.minSpeed = 20, .earlyExitRange = 8});
    chassis.moveToPoint(-3.1, -45.6, 1400, {.minSpeed = 28, .earlyExitRange = 6});
    chassis.turnToPoint(29.76, -33.23, 1000);
    chassis.waitUntilDone();
    doinker.set_value(true);
    chassis.moveToPoint(29.76, -33.23, 1000);
    chassis.turnToHeading(-24, 600, {.minSpeed = 100, .earlyExitRange = 5});
    chassis.swingToHeading(-40, DriveSide::LEFT, 500);
    chassis.waitUntilDone();
    intake_on = false;
    first_stage = true;
    intake2.move(127);
    chassis.moveToPose(9.1, 0.26, -45, 2000, {.lead = 0.15, .maxSpeed = 50});
    chassis.waitUntilDone();
    
    
    //doinker.set_value(false);
    leftMotors.move(60);
    rightMotors.move(70);
    pros::delay(350);
    intake2.move(27);
    intake1.move(-127);
    /*chassis.swingToHeading(13, DriveSide::LEFT, 1000, {.minSpeed = 30, .earlyExitRange = 9});
    chassis.moveToPose(-7.87, -4.6, -41, 1500, {.lead = 0.37, .minSpeed = 20, .earlyExitRange = 18});
    chassis.moveToPoint(-23, 11.34, 1700, {.maxSpeed = 50});
    chassis.moveToPoint(-17.8, 5.7, 1000, {.forwards = false});
    chassis.turnToPoint(-28.26, -3.1, 1000, {.minSpeed = 30, .earlyExitRange = 8});
    chassis.moveToPoint(-28.26, -3.1, 1000);
    chassis.waitUntil(1);
    targetTheta = 180;
    exitRange = 20;*/
}
void blue_ring() {}
void red_goal(){
    team_color = 'R';
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 5, 1000);
    targetTheta = 220;
    exitRange = 5;
    pros::delay(350);
    chassis.moveToPoint(0, -3, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 3});
    //chassis.moveToPoint(27.018, 11.853, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 10});
    chassis.moveToPoint(16.7, -32.9, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(9);
    chassis.cancelMotion();
    chassis.moveToPoint(16.7, -32.9, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 30});
    clampOn = true;
    while(!current){
        pros::delay(10);
    }
    targetTheta = 0;
    exitRange = 30;
    pros::delay(120);
    chassis.cancelMotion();
    chassis.moveToPose(-26.85, -20.7, -117, 1000, {.lead = 0.4, .minSpeed = 30});
    chassis.waitUntilDone();
    doinker.set_value(true);
    chassis.moveToPoint(-35.26, -24.64, 2000, {.maxSpeed = 60});
    chassis.turnToHeading(-210, 2000, {.minSpeed = 120});
    chassis.waitUntilDone();
    intake_on = true;
    chassis.moveToPoint(-26.5, -36.5, 1000);
    chassis.moveToPoint(-30.5, -30.79, 1000, {.forwards = false});
    chassis.waitUntil(2);
    doinker.set_value(false);
    chassis.turnToPoint(-5.28, -45.34, 1000);
    chassis.waitUntilDone();
    targetTheta = 26;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    exitRange = 0.2;
    chassis.moveToPoint(-5.28, -45.34, 1000);
    chassis.waitUntil(4);
    clampOn = false;
    while(!is_stuck){
        pros::delay(10);
    }
    intake_on = false;
    chassis.waitUntilDone();
    doinker.set_value(true);
    chassis.turnToHeading(-171, 1000, {.minSpeed = 40, .earlyExitRange = 10});
    chassis.waitUntilDone();
    doinker.set_value(false);
    chassis.turnToPoint(-3.41, -64.1, 1000);
    chassis.moveToPoint(-3.41, -64.1, 1000, {.maxSpeed = 50});
    chassis.waitUntilDone();
    targetTheta = 140;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    exitRange = 20;
}
void blue_goal(){
    team_color = 'B';
    chassis.setPose(0, 0, 0);
    pros::delay(1000);
    chassis.moveToPoint(0, 5, 1000);
    targetTheta = 220;
    exitRange = 5;
    pros::delay(350);
    chassis.moveToPoint(0, -3, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 3});
    //chassis.moveToPoint(27.018, 11.853, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 10});
    chassis.moveToPoint(-16.7, -32.9, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(9);
    targetTheta = 0;
    exitRange = 20;
    chassis.cancelMotion();
    chassis.moveToPoint(-16.7, -32.9, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 30});
    clampOn = true;
    while(!current){
        pros::delay(10);
    }
    pros::delay(50);
    chassis.cancelMotion();
    chassis.swingToPoint(1, -23.3, DriveSide::RIGHT, 1000, {.minSpeed = 40, .earlyExitRange = 4});
    chassis.waitUntilDone();
    
    doinker.set_value(true);
    chassis.moveToPoint(1, -23.3, 1000, {.minSpeed = 50, .earlyExitRange = 4});

    chassis.moveToPoint(31.1, -26.6, 1000, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 9});
    chassis.swingToHeading(50.5, DriveSide::LEFT, 500, {.minSpeed = 80, .earlyExitRange = 8});
    chassis.swingToHeading(12.5, DriveSide::RIGHT, 1000, {.minSpeed = 50, .earlyExitRange = 8});
    chassis.swingToHeading(-45, DriveSide::LEFT, 1000, {.minSpeed = 50, .earlyExitRange = 8});
    chassis.waitUntilDone();
    intake_on = true;
    chassis.moveToPoint(19.6, -20.7, 4000, {.maxSpeed = 70});
    chassis.waitUntil(1);
    doinker.set_value(false);
    chassis.waitUntilDone();
    pros::delay(1250);
    chassis.moveToPoint(27.7, -27.1, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 1});
    chassis.moveToPose(-2.76, -44.25, -185, 3000, {.lead = 0.2, .minSpeed = 50, .earlyExitRange = 8});
    chassis.waitUntil(10);
    clamp.set_value(false);
    targetTheta = 26;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    exitRange = 0.2;

    chassis.moveToPoint(-2.2, -66.12, 1000);     
    while(!is_stuck){
        pros::delay(10);
    }
    intake_on = false;
    first_stage = true;
    intake2.move(120);
    chassis.turnToHeading(-180, 1000);
    chassis.waitUntil(2);
    targetTheta = 150;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    exitRange = 20;
}
void skills_auto_v2(){
    team_color = 'R';
    chassis.setPose(0, 0, 0);
    targetTheta = 220;
    exitRange = 5;
    pros::delay(400);
    chassis.moveToPoint(0, -9, 700, {.forwards = false});
    chassis.turnToPoint(18.8, -6.7, 500, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(50);
    clampOn = true;
    chassis.moveToPoint(18.8, -6.7, 1000, {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
    while(!current){
        pros::delay(10);
    }
    chassis.cancelMotion();
    targetTheta = 0;
    exitRange = 40;
    chassis.turnToPoint(21.5, -25.37, 700);
    chassis.waitUntilDone();
    intake_on = true;
    chassis.moveToPose(21.5, -25.37, -218, 1000, {.lead = 0.25});
    chassis.moveToPoint(31.4, -39.14, 1000, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.swingToPoint(45.17, -73.9, DriveSide::RIGHT, 600, {.minSpeed = 40});
    chassis.waitUntilDone();
    targetTheta = 26;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    exitRange = 0.2;
    chassis.moveToPoint(45.17, -73.9, 1400);
    while(!is_stuck){
        pros::delay(10);
    }
    intake_on = false;
    targetTheta = 60;
    chassis.moveToPose(42.6, -53.5, -194, 1000, {.forwards = false, .lead = 0.2});
    chassis.turnToHeading(-271, 700);
    chassis.waitUntilDone();
    intake_on = true;
    chassis.moveToPoint(59.83, -53.65, 1000, {.minSpeed = 50});
    chassis.waitUntilDone();
    sequenceStep = 2;
    targetTheta = 220;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    exitRange = 30;
    pros::delay(350);
    leftMotors.move(-40);
    rightMotors.move(-40);
    pros::delay(500);
    leftMotors.move(0);
    rightMotors.move(0);
    //chassis.moveToPoint(45.55, .78, 1000, {.forwards = false});
    chassis.turnToPoint(47, 5.86, 1000);
    chassis.moveToPoint(47, 5.86, 1000, {.maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 9});
    chassis.moveToPoint(46.74, 7.38, 1000, {.minSpeed = 30});
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.setPose(float(reset_distance.get_distance()) / 100, 0, 0);
    chassis.moveToPose(-7.73, -13.5, 92.5, 1200, {.forwards = false, .lead = 0.2});
    chassis.moveToPoint(10.73, -13.54, 1000);
    chassis.swingToHeading(242, DriveSide::LEFT, 1000, {.minSpeed = 20}, false);
    chassis.waitUntilDone();
    intake_on = false;
    
    leftMotors.move(-70);
    rightMotors.move(-70);
    pros::delay(400);
    leftMotors.move(0);
    rightMotors.move(0);
    clampOn = false;
    pros::delay(80);
    chassis.moveToPose(-31.1, -11.877, 272.5, 1300, {.lead = 0.25, .maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(300);
    //chassis.setPose(fabs(cos(chassis.getPose().theta - 180) * float(reset_distance.get_distance())), chassis.getPose().y, chassis.getPose().theta);
    //pros::delay(50);
    chassis.turnToPoint(-60.55, -17.2, 1000, {.forwards = false});
    chassis.waitUntilDone();
    clampOn = true;
    chassis.moveToPoint(-68.3, -14.2, 1000, {.forwards = false, .maxSpeed = 60});
    while(!current){
        pros::delay(10);
    }
    chassis.cancelMotion();
    chassis.turnToPoint(-64.63, -37.1, 700);
    chassis.waitUntilDone();
    intake_on = true;
    chassis.moveToPoint(-64.63, -37.1, 1000);
    chassis.turnToHeading(577, 700);
    chassis.moveToPose(-84.67, -90.6, 190, 2000, {.lead = 0.2});
    chassis.waitUntil(13);
    targetTheta = 26;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    exitRange = 0.2;
    while(!is_stuck || !(temp > 100)){
        pros::delay(10);
        temp = temp + 1;
    }
    temp = 0;
    intake_on = false;
    targetTheta = 60;
    chassis.moveToPose(-83, -62.6, 210, 1200, {.forwards = false, .lead = 0.25});
    chassis.turnToPoint(-99, -64.8, 700);
    chassis.waitUntilDone();
    intake_on = true;
    chassis.moveToPoint(-99, -64.8, 1000, {.minSpeed = 20});
    chassis.waitUntilDone();
    sequenceStep = 2;
    targetTheta = 210;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    exitRange = 40;
    pros::delay(350);
    leftMotors.move(-40);
    rightMotors.move(-40);
    pros::delay(600);
    leftMotors.move(0);
    rightMotors.move(0);
    //chassis.moveToPoint(-83.9, -58.4, 1000, {.forwards = false});
   
    chassis.turnToPoint(-87, -0.88, 700);
    chassis.moveToPoint(-87, -0.88, 2000, {.maxSpeed = 50});
    chassis.moveToPoint(-87, -19, 1000, {.forwards = false, .minSpeed = 20});
    chassis.turnToHeading(630, 1000);
    chassis.waitUntilDone();
    clampOn = true;
    while(!current){
        pros::delay(10);
    }
    pros::delay(100);


    leftMotors.move(80);
    rightMotors.move(80);
    pros::delay(1000);
    leftMotors.move(0);
    rightMotors.move(0);
    chassis.setPose(float(reset_distance.get_distance()) / 25.4, 0, 0);
    pros::delay(50);
    chassis.swingToHeading(-135, DriveSide::RIGHT, 1000, {.minSpeed = 20});
    chassis.waitUntilDone();
    leftMotors.move(-50);
    rightMotors.move(-50);
    pros::delay(300);
    clampOn = false;
    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
    chassis.turnToHeading(-137,  1000, {.minSpeed = 40});
    chassis.waitUntilDone();
    intake_on = false;
    intake2.move(127);
    chassis.moveToPoint(-52.5, -96.7, 5000, {.maxSpeed = 100});
    chassis.turnToPoint(-77.6, -90.1, 1000, {.forwards = false});
    chassis.waitUntilDone();
    ringStop = true;
    intake_on = true;
    
    clampOn = true;
    chassis.moveToPoint(-77.6, -90.1, 1200, {.forwards = false, .maxSpeed = 60, .minSpeed = 9});
    while(!current){
        pros::delay(10);
    }
    chassis.cancelMotion();
    chassis.turnToPoint(-80.25, -109.67, 1000);
    chassis.waitUntilDone();
    first_stage = true;
    intake2.move(127);
    doinker.set_value(true);
    chassis.moveToPoint(-80.25, -109.67, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(-83.46, -119.3, -180, 1000, {.lead = 0.2});
    chassis.turnToHeading(-260, 1400, {.minSpeed = 100, .earlyExitRange = 10});
    chassis.turnToHeading(-320, 1000);
    chassis.waitUntilDone();
    clampOn = false;
    leftMotors.move(-50);
    rightMotors.move(-50);
    pros::delay(300);
    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(500);
    leftMotors.move(0);
    rightMotors.move(0);
    chassis.turnToPoint(-81.44, -103.5, 1000, {.forwards = false});
    chassis.waitUntilDone();
    doinker.set_value(false);
    clampOn  = true;
    chassis.moveToPoint(-81.44, -103.5, 1500, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.turnToPoint(-68.45, -62.1, 700, {.forwards = false});
    chassis.waitUntilDone();
    
    targetTheta = 26;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    exitRange = 0.2;
    chassis.moveToPoint(-68.45, -62.1, 1500, {.forwards = false, .maxSpeed = 50});
    chassis.turnToPoint(-80.76, -65.2, 700);
    chassis.waitUntilDone();
    ringStop = false;
    first_stage = false;
    intake_on = true;
    while(!is_stuck){
        pros::delay(10);
    }
    intake_on = false;
    leftMotors.move(100);
    rightMotors.move(100);
    pros::delay(400);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(100);
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -7.4, 800, {.forwards = false, .maxSpeed = 30});
    chassis.waitUntilDone();
    targetTheta = 220;
    exitRange = 10;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(100);
    intake_on = true;
    pros::delay(350);
    chassis.moveToPoint(0, -16, 800, {.forwards = false, .maxSpeed = 30});
    chassis.turnToPoint(23.865, -6.147, 1000);
    chassis.waitUntilDone();
    targetTheta = 0;
    exitRange = 30;
    clampOn = false;
    leftMotors.move(120);
    rightMotors.move(120);
    pros::delay(1500);
    leftMotors.move(0);
    rightMotors.move(0);
    chassis.turnToHeading(45, 1000);
    chassis.waitUntilDone();
    leftMotors.move(-60);
    rightMotors.move(-60);
    hang.set_value(true);
    pros::delay(200);
    targetTheta = 41;
    lb1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    exitRange = 1;
    pros::delay(1600);
    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(120);
    leftMotors.move(0);
    rightMotors.move(0);
}   
void test(){
    team_color = 'R';
    clampOn = true;
    while(!current){
        pros::delay(10);
    }
    intake_on = true;
}

rd::Selector selector({
    {"Red SAWP", &red_SAWP},
    {"Blue SAWP", &blue_SAWP},
    {"Red Goal", &red_goal},
    {"Blue Goal", &blue_goal},
    {"Red Ring", &red_ring},
    {"Blue Ring", &blue_ring},
    {"Skills Auto", &skills_auto_v2}
});

rd::Console console;
rd_view_t *rd_view_create(const char *name);
void rd_view_focus(rd_view_t *name);


void initialize() {
    //pros::lcd::initialize();
    selector.focus();
    chassis.calibrate();  // Wait for calibration before starting tasks
    

    // Start background tasks only after calibration
    pros::Task screenTask([]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            
            //pros::lcd::print(4, "%i", line_tracker.get_value());
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });

    // Start background tasks
    pros::Task isStuckTask(is_stuck_check, nullptr, "stuck task");
    pros::Task clampTask(clampCheck, nullptr, "clamp task");
    pros::Task color_check_task(color_check, nullptr, "color check task");
    

    pros::Task lbPidTask(LBpidTask, nullptr, "LB PID Task");
}


void disabled() {}

void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function

void autonomous() {
    optical.set_led_pwm(95);
    selector.run_auton();
    //red_ring();
}


/**
 * Runs in driver control
 */
void opcontrol() {
    pros::Task hangTask(hang_task, nullptr, "hang task");
    //pros::Task lbPidTask(LBpidTask, nullptr, "LB PID Task");
    

    optical.set_led_pwm(0);

    
    auton = false;

    bool doinkerState = false;

    ringStop = false;

    targetTheta = 0;
    exitRange = 30;

    /*  chassis.setPose(0, 0, 0);
    targetTheta = 220;
    exitRange = 5;
    pros::delay(400);
    chassis.moveToPoint(0, -9, 700, {.forwards = false});
    chassis.turnToPoint(18.8, -6.7, 500, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(50);
    clampOn = true;
    chassis.moveToPoint(18.8, -6.7, 1000, {.forwards = false, .maxSpeed = 70, .minSpeed = 40});
    chassis.waitUntilDone();
    targetTheta = 0;
    exitRange = 31;
    controller.rumble(".");*/
    opC = true;
    while (true) {

        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        float scaleFactor = current ? 0.45 : 0.22;
        chassis.arcade(leftY, rightX, false, scaleFactor);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            current = !current; // Toggle piston state
            clamp.set_value(current);
        } 

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            doinkerState = !doinkerState; // Toggle piston state
            doinker.set_value(doinkerState);
        } 


        // Intake control
        
        int intakeSpeed = 0;

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            reversed = false;
            if (!is_stuck){
                intake1.move(-127);
                intake2.move(127);
            }
            else{
                intake2.move(127);
            }
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake1.move(127);
            intake2.move(-127);
            reversed = true;
        }
        else{
            intake1.move(0);
            intake2.move(0);
        }

        pros::delay(10);  // Small delay to avoid overwhelming the system
}
}