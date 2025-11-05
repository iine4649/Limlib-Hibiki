#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/poos.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
\
#define LF_PORT 1
#define LB_PORT 16

#define RF_PORT 10
#define RB_PORT 18
#define IU_PORT 13
#define ID_PORT 20
// controller
// main.cpp
// A high-performance C++11 implementation of mixture Monte Carlo localization 
// using Box2D for collision and simulation, and three VEX-style distance sensors.
// BLASFEO calls can be inserted in computational hotspots (here in sensor likelihood).

#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <cassert>

// Box2D header (adjust include path as needed)
#include <box2d/box2d.h>

// BLASFEO header (if available; otherwise, use your BLASFEO include)
// #include "blasfeo_d_aux.h"

// ----- Assume these classes are provided -----
// Vector2 and Pose are assumed to be defined according to your description.
// (They include vector arithmetic, rotation, and conversion functions.)
#include "Vector2.h"
#include "Pose.h"

// ----- Additional structures and enums -----

enum class SensorPlacement { FRONT, LEFT, BACK, RIGHT };

struct Circle {
    Vector2 position;
    float radius_squared;
    float radius;
};

// ----- DistanceSensor class -----
class DistanceSensor {
public:
    float measurement;           // -1 if invalid, otherwise last measurement
    float noise_sigma;           // measurement standard deviation
    Vector2 position_on_robot;   // sensor offset from robot center
    SensorPlacement placement;   // sensor mounting location

    static const float MAX_RANGE; // maximum sensor range

    DistanceSensor(const Vector2 &pos, SensorPlacement place, float noise)
        : measurement(-1), noise_sigma(noise), position_on_robot(pos), placement(place) {}

    // Update the sensor measurement using a Box2D raycast.
    // (Implement your raycasting using the provided Box2D world and the robot's current pose.)
    void update_measurement(b2World &world, const Pose &robot_pose) {
        // Compute the sensor's global position by rotating the offset and adding robot_pose.
        double cos_theta = std::cos(robot_pose.theta);
        double sin_theta = std::sin(robot_pose.theta);
        Vector2 global_offset(robot_pose.x + position_on_robot.x * cos_theta - position_on_robot.y * sin_theta,
                               robot_pose.y + position_on_robot.x * sin_theta + position_on_robot.y * cos_theta);

        // Determine the sensor’s ray direction based on its placement.
        double sensor_angle = 0.0;
        switch (placement) {
            case SensorPlacement::FRONT: sensor_angle = robot_pose.theta; break;
            case SensorPlacement::LEFT:  sensor_angle = robot_pose.theta + M_PI_2; break;
            case SensorPlacement::BACK:  sensor_angle = robot_pose.theta + M_PI; break;
            case SensorPlacement::RIGHT: sensor_angle = robot_pose.theta - M_PI_2; break;
        }

        // Set up the ray start and end points.
        b2Vec2 ray_start(global_offset.x, global_offset.y);
        b2Vec2 ray_dir(std::cos(sensor_angle), std::sin(sensor_angle));
        b2Vec2 ray_end = ray_start + DistanceSensor::MAX_RANGE * ray_dir;

        // (Perform a Box2D raycast here. For simplicity, we set a dummy measurement.)
        // In your full implementation, you would implement a custom b2RayCastCallback.
        // For now, we assume the sensor sees the wall at a distance equal to MAX_RANGE/2.
        measurement = DistanceSensor::MAX_RANGE / 2.0f;
        
        // Add noise
        measurement += random_gaussian(0, noise_sigma);
        // If the measurement is beyond MAX_RANGE, mark it as invalid.
        if (measurement > MAX_RANGE)
            measurement = -1;
    }

    float get_measurement() const {
        return measurement;
    }
    
    bool is_valid_measurement() const {
        return measurement >= 0 && measurement <= MAX_RANGE;
    }

private:
    // Simple Gaussian noise generator.
    float random_gaussian(float mean, float stddev) {
        static std::default_random_engine eng;
        std::normal_distribution<float> dist(mean, stddev);
        return dist(eng);
    }
};

const float DistanceSensor::MAX_RANGE = 500.0f; // Example maximum range

// ----- Field class -----
class Field {
public:
    float size;       // side length of the square field
    float half_size;  // half the side length
    std::vector<Circle> obstructions; // obstacles (if any)

    Field(float s) : size(s), half_size(s / 2.0f) {}

    // Compute the expected sensor measurement for a given particle.
    // (This simple version computes the distance to the field boundary.)
    float expected_measurement(const struct Particle &particle, const DistanceSensor &sensor) const {
        // Compute sensor's global position using particle pose.
        double cos_theta = std::cos(particle.pose.theta);
        double sin_theta = std::sin(particle.pose.theta);
        Vector2 global_offset(particle.pose.x + sensor.position_on_robot.x * cos_theta - sensor.position_on_robot.y * sin_theta,
                              particle.pose.y + sensor.position_on_robot.x * sin_theta + sensor.position_on_robot.y * cos_theta);

        // Determine the sensor ray direction.
        double sensor_angle = 0.0;
        switch (sensor.placement) {
            case SensorPlacement::FRONT: sensor_angle = particle.pose.theta; break;
            case SensorPlacement::LEFT:  sensor_angle = particle.pose.theta + M_PI_2; break;
            case SensorPlacement::BACK:  sensor_angle = particle.pose.theta + M_PI; break;
            case SensorPlacement::RIGHT: sensor_angle = particle.pose.theta - M_PI_2; break;
        }
        // For simplicity, compute intersection with the field boundaries.
        // (A full raycast with obstructions would be more complex.)
        // Here we assume the field is a square centered at (0,0).
        // Compute distances to the four boundaries and return the smallest positive value.
        double dx = (sensor_angle >= -M_PI_2 && sensor_angle <= M_PI_2) ?
            (half_size - global_offset.x) : (global_offset.x + half_size);
        double dy = (sensor_angle >= 0 && sensor_angle <= M_PI) ?
            (half_size - global_offset.y) : (global_offset.y + half_size);
        // Approximate the distance along the ray.
        double distance = std::min(dx / std::abs(std::cos(sensor_angle) + 1e-6),
                                   dy / std::abs(std::sin(sensor_angle) + 1e-6));
        return static_cast<float>(distance);
    }
};

// ----- Particle struct -----
struct Particle {
    lemlib::Pose pose;    // current pose estimate
    float weight; // importance weight
};

// ----- MixtureMCL class -----
// Implements the four steps: motion update, dual sampling, importance weighting, and resampling.
class MixtureMCL {
public:
    MixtureMCL(int num_particles, const Field &field)
        : _num_particles(num_particles), _field(field)
    {
        initialize_particles();
    }
    
    // Apply the motion model (with noise) to each particle.
    void motion_update(const Pose &delta_pose) {
        for (auto &particle : _particles) {
            particle.pose = apply_motion_model(particle.pose, delta_pose);
        }
    }
    
    // Dual sampling: here we approximate sampling from the observation model by
    // slightly perturbing each particle. (A real implementation would use a KD‑tree.)
    void dual_sampling(const std::vector<float> & /*observation*/) {
        for (auto &particle : _particles) {
            particle.pose.x += random_gaussian(0, _dual_sampling_noise_std);
            particle.pose.y += random_gaussian(0, _dual_sampling_noise_std);
            particle.pose.theta += random_gaussian(0, _dual_sampling_noise_std_angle);
        }
    }
    
    // Compute importance weights for each particle using the sensor likelihood.
    // (This is where BLASFEO routines could be applied to vectorize the per‑particle likelihood
    // calculations.)
    void importance_weighting(const std::vector<DistanceSensor> &sensors,
                                const std::vector<float> &observation)
    {
        float weight_sum = 0.0f;
        for (auto &particle : _particles) {
            float likelihood = 1.0f;
            // For each sensor, compute the likelihood given the expected measurement.
            for (size_t i = 0; i < sensors.size(); ++i) {
                float expected = _field.expected_measurement(Particle{particle.pose, particle.weight}, sensors[i]);
                float measured = observation[i];
                float sensor_likelihood = compute_sensor_likelihood(measured, expected, sensors[i].noise_sigma);
                likelihood *= sensor_likelihood;
            }
            particle.weight = likelihood;
            weight_sum += particle.weight;
        }
        // Normalize weights.
        for (auto &particle : _particles) {
            particle.weight /= weight_sum;
        }
    }
    
    // Resample particles using systematic resampling.
    void resample_particles() {
        std::vector<Particle> new_particles;
        new_particles.reserve(_num_particles);
        std::vector<float> cumulative(_num_particles, 0.0f);
        cumulative[0] = _particles[0].weight;
        for (int i = 1; i < _num_particles; ++i) {
            cumulative[i] = cumulative[i - 1] + _particles[i].weight;
        }
        float step = 1.0f / _num_particles;
        float start = random_uniform(0, step);
        int index = 0;
        for (int i = 0; i < _num_particles; ++i) {
            float u = start + i * step;
            while (u > cumulative[index])
                index++;
            new_particles.push_back(_particles[index]);
        }
        _particles = new_particles;
    }
    
    // One full iteration of Mixture-MCL.
    void iteration(const Pose &delta_pose, const std::vector<DistanceSensor> &sensors,
                   const std::vector<float> &observation)
    {
        motion_update(delta_pose);
        dual_sampling(observation);
        importance_weighting(sensors, observation);
        resample_particles();
    }
    
    // Compute the estimated pose as the weighted average of particle poses.
    Pose get_estimated_pose() const {
        double sum_x = 0, sum_y = 0, sum_theta = 0;
        for (const auto &particle : _particles) {
            sum_x += particle.pose.x * particle.weight;
            sum_y += particle.pose.y * particle.weight;
            sum_theta += particle.pose.theta * particle.weight;
        }
        return Pose(sum_x, sum_y, sum_theta);
    }
    
private:
    int _num_particles;
    std::vector<Particle> _particles;
    const Field &_field;
    
    // Dual sampling noise parameters (tunable)
    const float _dual_sampling_noise_std = 0.01f;
    const float _dual_sampling_noise_std_angle = 0.005f;
    
    // Motion noise parameters.
    const float _motion_noise_std = 0.02f;
    const float _motion_noise_std_angle = 0.01f;
    
    // Initialize particles uniformly over the field.
    void initialize_particles() {
        std::default_random_engine eng;
        std::uniform_real_distribution<float> pos_dist(-_field.half_size, _field.half_size);
        std::uniform_real_distribution<float> angle_dist(-M_PI, M_PI);
        _particles.resize(_num_particles);
        for (auto &particle : _particles) {
            particle.pose = Pose(pos_dist(eng), pos_dist(eng), angle_dist(eng));
            particle.weight = 1.0f / _num_particles;
        }
    }
    
    // Apply the motion model (delta_pose in the robot’s local frame is converted to global motion).
    Pose apply_motion_model(const Pose &pose, const Pose &delta) {
        double cos_theta = std::cos(pose.theta);
        double sin_theta = std::sin(pose.theta);
        double global_dx = delta.x * cos_theta - delta.y * sin_theta;
        double global_dy = delta.x * sin_theta + delta.y * cos_theta;
        double noise_x = random_gaussian(0, _motion_noise_std);
        double noise_y = random_gaussian(0, _motion_noise_std);
        double noise_theta = random_gaussian(0, _motion_noise_std_angle);
        return Pose(pose.x + global_dx + noise_x,
                    pose.y + global_dy + noise_y,
                    pose.theta + delta.theta + noise_theta);
    }
    
    // Gaussian likelihood for a sensor measurement.
    float compute_sensor_likelihood(float measured, float expected, float sigma) {
        float diff = measured - expected;
        float exponent = - (diff * diff) / (2 * sigma * sigma);
        float likelihood = (1.0f / (sigma * std::sqrt(2 * M_PI))) * std::exp(exponent);
        return likelihood;
    }
    
    // Random number generators.
    float random_gaussian(float mean, float stddev) {
        static std::default_random_engine eng;
        std::normal_distribution<float> dist(mean, stddev);
        return dist(eng);
    }
    
    float random_uniform(float a, float b) {
        static std::default_random_engine eng;
        std::uniform_real_distribution<float> dist(a, b);
        return dist(eng);
    }
};

// ----- Simulation class -----
// Creates a Box2D world representing a square field with boundaries,
// a robot body, and integrates three distance sensors.
// It then runs a simulation loop that applies the motion model,
// updates sensor measurements, and iterates the Mixture-MCL.
class Simulation {
public:
    Simulation()
        : _world(b2Vec2(0.0f, 0.0f)),
          _field(10.0f),         // e.g., a 10x10 field
          _mcl(1000, _field)      // initialize with 1000 particles
    {
        create_boundaries();
        create_robot();
        init_sensors();
    }
    
    void run() {
        for (int step = 0; step < SIMULATION_STEPS; ++step) {
            // Simulate robot motion (here a constant forward move with slight rotation).
            Pose delta = get_robot_motion();
            update_robot(delta);
            
            // Update sensor measurements.
            std::vector<float> sensor_measurements;
            for (auto &sensor : _sensors) {
                sensor.update_measurement(_world, _robot_pose);
                sensor_measurements.push_back(sensor.get_measurement());
            }
            
            // Run one iteration of Mixture-MCL.
            _mcl.iteration(delta, _sensors, sensor_measurements);
            Pose estimated = _mcl.get_estimated_pose();
            std::cout << "Step " << step << ": Estimated Pose: (" 
                      << estimated.x << ", " << estimated.y << ", " 
                      << estimated.theta << ")\n";
            
            // Step the Box2D world.
            _world.Step(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);
        }
    }
    
private:
    b2World _world;
    Field _field;
    MixtureMCL _mcl;
    Pose _robot_pose;
    std::vector<DistanceSensor> _sensors;
    b2Body* _robot_body;
    
    static constexpr int SIMULATION_STEPS = 1000;
    static constexpr float TIME_STEP = 1.0f / 60.0f;
    static constexpr int VELOCITY_ITERATIONS = 8;
    static constexpr int POSITION_ITERATIONS = 3;
    
    // Create the field boundaries in Box2D.
    void create_boundaries() {
        b2BodyDef groundBodyDef;
        groundBodyDef.position.Set(0.0f, 0.0f);
        b2Body* groundBody = _world.CreateBody(&groundBodyDef);
        
        b2EdgeShape edge;
        // Bottom
        edge.Set(b2Vec2(-_field.half_size, -_field.half_size),
                 b2Vec2(_field.half_size, -_field.half_size));
        groundBody->CreateFixture(&edge, 0.0f);
        // Top
        edge.Set(b2Vec2(-_field.half_size, _field.half_size),
                 b2Vec2(_field.half_size, _field.half_size));
        groundBody->CreateFixture(&edge, 0.0f);
        // Left
        edge.Set(b2Vec2(-_field.half_size, -_field.half_size),
                 b2Vec2(-_field.half_size, _field.half_size));
        groundBody->CreateFixture(&edge, 0.0f);
        // Right
        edge.Set(b2Vec2(_field.half_size, -_field.half_size),
                 b2Vec2(_field.half_size, _field.half_size));
        groundBody->CreateFixture(&edge, 0.0f);
    }
    
    // Create the robot as a dynamic body in Box2D.
    void create_robot() {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(0.0f, 0.0f);
        _robot_body = _world.CreateBody(&bodyDef);
        b2PolygonShape box;
        box.SetAsBox(0.5f, 0.5f); // robot dimensions
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &box;
        fixtureDef.density = 1.0f;
        fixtureDef.friction = 0.3f;
        _robot_body->CreateFixture(&fixtureDef);
        _robot_pose = Pose(0.0, 0.0, 0.0);
    }
    
    // Initialize three distance sensors (FRONT, LEFT, RIGHT).
    void init_sensors() {
        _sensors.emplace_back(Vector2(0.5, 0.0), SensorPlacement::FRONT, 0.05f);
        _sensors.emplace_back(Vector2(0.0, 0.5), SensorPlacement::LEFT, 0.05f);
        _sensors.emplace_back(Vector2(0.0, -0.5), SensorPlacement::RIGHT, 0.05f);
    }
    
    // Example robot motion: constant forward move with slight rotation.
    Pose get_robot_motion() {
        return Pose(0.1, 0.0, 0.01);
    }
    
    // Update the robot's pose in the simulation.
    void update_robot(const Pose &delta) {
        _robot_pose = _robot_pose + delta;
        _robot_body->SetTransform(b2Vec2(_robot_pose.x, _robot_pose.y), _robot_pose.theta);
    }
};

int main() {
    Simulation sim;
    sim.run();
    return 0;
}


// motor groups
pros::Controller controller1(pros::E_CONTROLLER_MASTER);
pros::Motor Intake(-8,pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-5, -20,14},pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-10, 7,2}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros::Motor lbL(-12);
pros::Motor lbR(13);
pros::Rotation lbRot(17);

// motors
pros::adi::Pneumatics BackC('A',false);
pros::adi::Pneumatics doinkerL('C',false);
pros::adi::Pneumatics doinkerR('B',false);
pros::adi::Pneumatics intakeR('D',false);
pros::Optical americanPolice(16);
/*
pros::Motor IntakeU(13);
pros::Motor IntakeD(20);*/
// Inertial Sensor on port 10
pros::Imu imu(19);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-1);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(4);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -2.72);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -0.051);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.817, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            12, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             100, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
    bool redRace = 1;
    bool blueRace = 0;
    double intakeVel = 127;
 void activeRacism(){
    if(redRace){
        if(130 < americanPolice.get_hue() && americanPolice.get_hue() < 350){
            pros::delay(200);
            intakeVel = -127;
            //Intake.brake();
            pros::delay(750);
            intakeVel = 127;
        }
    }
    if(blueRace){
        if(0 < americanPolice.get_hue() && americanPolice.get_hue() < 20){
            pros::delay(200);
            intakeVel = -127;
            //Intake.brake();
            pros::delay(750);
            intakeVel = 127;
        }
    }
 }
 
 int currState = 0;

 bool special = 0;
 void specialState(){
    if(special){
        special = 0;
        currState = 0;
    } else {
        special = 1;
        currState = 3;
    }
 }
 const int numStates = 3;
 //make sure these are in centidegrees (1 degree = 100 centidegrees)
 int states[6] = {500, 3650, 18000, 23000,19000,15000};
 int target = 0;
 
 void nextState() {
     currState += 1;
     if (currState >= numStates) {
      
        currState = 0;
     }
 }

 
 void liftControl() {
     target = states[currState];
     double kp = 0.008;
     double kd = 0.0005; 
     double ki = 0.001;
     double error = target - lbRot.get_position();
     double preverror; 
     double derivative = error - preverror;
     preverror = error;
     double totalerror;
     totalerror += error;
     double velocity = kp * error + kd * derivative + ki * totalerror;
     lbL.move(velocity);
     lbR.move(velocity);
 }

 void antiStuck(){
    if(Intake.get_actual_velocity()< 10 &&Intake.get_actual_velocity()> 0){
        intakeVel =-127;
        pros::delay(750);
        intakeVel = 127;
    }
 }
 
void initialize() {
    //pros::lcd::initialize(); // initialize brain screen
    loading();
    chassis.calibrate(); // calibrate sensors
    drawGUI();
    pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            //activeRacism();    // infinite loop
               // print measurements from the adi encoder
            //ros::lcd::print(0, "Horizontal Encoder: %i", horizontalEnc.get_position());
                // print measurements from the rotation sensor
            //pros::lcd::print(1, "Vertical Encoder: %i", verticalEnc.get_position());
            //pros::lcd::print(3, "IntakeVel: %i", intakeVel);
            //pros::lcd::print(4, "colour: %i", americanPolice.get_hue());

            pros::delay(10);
        }
    });
/*
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57), LV_PART_MAIN);
*/
/*Create a white label, set its text and align it to the center*/
  /*  lv_obj_t * label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello world");
    lv_obj_set_style_text_color(label, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);*/
    
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    /*
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });*/
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy
ASSET(firstmove_txt);
ASSET(second_txt);
ASSET(ClampToLB_txt);
ASSET(AWP1_txt);
ASSET(AWP2_txt);
ASSET(firstmoveblue_txt);
ASSET(ClampToLBBlue_txt);
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
/*
#1 Red Positive
#2 Blue Positive two top rings goal rush
#3 Blue Positive two top rings goal rush
#4 Red Positive 8889X



*/
void redNegAuton(){
    //red neg 
    
    //chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    americanPolice.set_led_pwm(100);
    chassis.setPose(-62.712, 36.074, 270);
    chassis.moveToPoint(-22,26,750,{.forwards = false,.maxSpeed = 80, .minSpeed = 40}); ///xijuku no happiness
    chassis.waitUntilDone();
    pros::delay(700);
    BackC.extend();
    Intake.move(127);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(-24,48,1750,{.maxSpeed = 60, .minSpeed=40});
    chassis.waitUntilDone();
    pros::delay(5000);
    currState=5;
    /*chassis.waitUntilDone();
    intakeR.extend();
    chassis.moveToPoint(-48.981,-0,1750,{.maxSpeed = 60});
    chassis.waitUntilDone();
    intakeR.retract();
    pros::delay(500);
    chassis.moveToPoint(-48,15,1750,{.forwards = false});
    chassis.moveToPoint(-48,-10,1750,{.forwards = true});
    pros::delay(2000);*/
    chassis.moveToPoint(-24,0,1750,{.forwards = true});
    //chassis.moveToPoint(-35.981,-20,1750);

    /*Intake.move(intakeVel);
    chassis.follow(firstmove_txt,17.5,1000);
    chassis.waitUntilDone();
    chassis.turnToPoint(-10, 47, 750,{.earlyExitRange = 2});
    chassis.waitUntilDone();
    doinkerL.extend();
    pros::delay(250);
    chassis.moveToPoint(-15,42,750,{.maxSpeed = 127, .minSpeed = 120}); ///xijuku no happiness
    pros::delay(100);
    doinkerL.extend();
    chassis.waitUntilDone();
    pros::delay(700);
    Intake.brake();
    chassis.moveToPoint(-31,27,1750,{.forwards=false,.maxSpeed = 60, .minSpeed=40});
    chassis.waitUntilDone();
    BackC.extend();
    Intake.move(intakeVel);
    doinkerL.retract();
    doinkerR.extend();
    pros::delay(500);
    chassis.swingToHeading(300,DriveSide::LEFT,500,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed = 84, .minSpeed = 60 });
    chassis.waitUntilDone();
    doinkerR.retract();
    chassis.moveToPoint(-23.788,42.223,1750,{.forwards = true});
    
    chassis.follow(ClampToLB_txt,15,3500);
    chassis.waitUntilDone();
    intakeR.extend();
    chassis.moveToPoint(-35.981,-12,1750,{.maxSpeed = 60});
    chassis.waitUntilDone();
    intakeR.retract();
    pros::delay(500);
    chassis.moveToPoint(-35.981,-3,1750,{.forwards = false});
    chassis.moveToPoint(-35.981,-10,1750,{.forwards = true});
    //chassis.moveToPoint(-35.981,-20,1750);
*/
}

void blueNegAuton(){
    /*
    chassis.setPose(62.712, 36.074, 270);
    
    Intake.move(127);
    chassis.follow(firstmoveblue_txt,17.5,1000);
    doinkerR.extend();
    chassis.moveToPoint(10,46.5,750); ///xijuku no happiness
    chassis.waitUntilDone();
    pros::delay(700);
    Intake.brake();
    chassis.moveToPoint(24,24,1750,{.forwards=false,.maxSpeed = 60, .minSpeed=40});
    chassis.waitUntilDone();
    BackC.extend();
    Intake.move(127);
    doinkerR.retract();
    doinkerL.extend();
    pros::delay(500);
    chassis.swingToHeading(60,DriveSide::RIGHT,500,{.direction = AngularDirection::CW_CLOCKWISE,.maxSpeed = 84, .minSpeed = 60 });
    chassis.waitUntilDone();
    doinkerL.retract();
    chassis.moveToPoint(23.788,42.223,1750,{.forwards = true});
    
    chassis.follow(ClampToLBBlue_txt,15,3500);
    chassis.waitUntilDone();
    intakeR.extend();
    chassis.moveToPoint(35.981,-12,1750);
    chassis.waitUntilDone();
    intakeR.retract();
    pros::delay(500);
    chassis.moveToPoint(35.981,-3,1750);
    */
    
    americanPolice.set_led_pwm(100);
    chassis.setPose(62.712, 36.074, 90);
    chassis.moveToPoint(24,24,750,{.forwards = false,.maxSpeed = 80, .minSpeed = 40}); ///xijuku no happiness
    chassis.waitUntilDone();
    pros::delay(700);
    BackC.extend();
    Intake.move(127);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(24,48,1750,{.maxSpeed = 60, .minSpeed=40});
    chassis.waitUntilDone();
    pros::delay(5000);
    currState=5;
    /*chassis.waitUntilDone();
    intakeR.extend();
    chassis.moveToPoint(-48.981,-0,1750,{.maxSpeed = 60});
    chassis.waitUntilDone();
    intakeR.retract();
    pros::delay(500);
    chassis.moveToPoint(-48,15,1750,{.forwards = false});
    chassis.moveToPoint(-48,-10,1750,{.forwards = true});
    pros::delay(2000);*/
    chassis.moveToPoint(24,0,1750,{.forwards = true});
}
//inspected
void awpRedNeg(){
    //AWP Neg RED
    
    currState = 5;
    chassis.setPose(-61, 8.92, 229.4);
    chassis.moveToPoint(-67,4.108,1000);
    chassis.waitUntilDone();
    currState = 4; 
    pros::delay(1000);
    chassis.moveToPoint(-61, 8.92, 750,{.forwards = false});
    chassis.moveToPoint(-30,24,1000,{.forwards = false,.maxSpeed = 75,.minSpeed = 40});
    chassis.waitUntilDone();
    currState=0;
    pros::delay(250);
    BackC.extend();
    Intake.move(intakeVel);
    //chassis.moveToPoint(-10.807,6.108,1000,{.forwards = false,.maxSpeed = 40,.minSpeed = 20});
    //chassis.waitUntilDone();
    //chassis.turnToHeading(350,500);
    
    //chassis.turnToPoint(10,39.76,1500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(-19.5,39,1500,{.maxSpeed = 127,.minSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(1000);
    //chassis.swingToHeading(225,DriveSide::LEFT,1500,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed = 127, .minSpeed = 80 });
    
    chassis.moveToPoint(-24,30,1500,{.forwards = false,.maxSpeed = 127,.minSpeed = 100});
    chassis.waitUntilDone();
    chassis.moveToPoint(-35,43,1500,{.maxSpeed = 127,.minSpeed = 100});
    chassis.waitUntilDone();
    chassis.moveToPoint(-40,-0,1500,{.maxSpeed = 127,.minSpeed = 100});
    chassis.waitUntil(50);
    BackC.retract();
    chassis.moveToPoint(-40,-3,500,{.maxSpeed = 127,.minSpeed = 60});
    /*pros::delay(1000);
    intakeR.retract();
    
    //pros::delay(1000);
    chassis.moveToPoint(-46,-0,1500,{.forwards = false, .maxSpeed = 127,.minSpeed = 60});
    chassis.waitUntilDone();
    BackC.retract();
    Intake.brake();
    chassis.moveToPoint(-46,-5,1500,{ .maxSpeed = 127,.minSpeed = 80});
    */
    chassis.turnToHeading(0,500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(-22,-26,1500,{.forwards = false,.maxSpeed = 80,.minSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(500);
    
    BackC.extend();
    Intake.move(127);
    chassis.moveToPoint(-24,-48,1500,{.maxSpeed = 127,.minSpeed = 100});
    pros::delay(750);
    chassis.moveToPoint(0,0,20000,{.maxSpeed = 127,.minSpeed = 100});
    currState = 5;
    pros::delay(500);
    Intake.brake();
    
    //chassis.moveToPoint(-6,31.673,1500,{.maxSpeed = 127,.minSpeed = 80});
    /*chassis.follow(AWP2_txt,17.5,2500);
    chassis.waitUntilDone();
    BackC.retract();
    chassis.turnToHeading(320.1,500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-24.807,-24.014,1000,{.forwards = false,.maxSpeed = 127,.minSpeed = 80});
    BackC.extend();*/

}

//inspected
void awpBlueNeg(){
    //AWP Neg BLUE
    
    currState = 5;
    chassis.setPose(61, 8.92, 139.4);
    chassis.moveToPoint(66,4.108,1000);
    

    chassis.waitUntilDone();
    currState = 4; 
    pros::delay(1000);
    chassis.moveToPoint(61, 8.92, 750,{.forwards = false});
    chassis.moveToPoint(30,24,1000,{.forwards = false,.maxSpeed = 75,.minSpeed = 40});
    chassis.waitUntilDone();
    currState=0;
    pros::delay(250);
    BackC.extend();
    //chassis.moveToPoint(-10.807,6.108,1000,{.forwards = false,.maxSpeed = 40,.minSpeed = 20});
    //chassis.waitUntilDone();
    //chassis.turnToHeading(350,500);
    
    //chassis.turnToPoint(10,39.76,1500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(19.5,39,1500,{.maxSpeed = 127,.minSpeed = 80});
    Intake.move(intakeVel);
    chassis.waitUntilDone();
    pros::delay(1000);
    //chassis.swingToHeading(225,DriveSide::LEFT,1500,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed = 127, .minSpeed = 80 });
    
    chassis.moveToPoint(24,30,1500,{.forwards = false,.maxSpeed = 127,.minSpeed = 100});
    chassis.waitUntilDone();
    chassis.moveToPoint(35,43,1500,{.maxSpeed = 127,.minSpeed = 100});
    chassis.waitUntilDone();
    currState = 5;
    //Intake.brake();
    chassis.moveToPoint(24,-5,1500,{.maxSpeed = 127,.minSpeed = 100});
    /*chassis.waitUntil(50);
    BackC.retract();
    chassis.moveToPoint(40,-3,500,{.maxSpeed = 127,.minSpeed = 60});*/
    /*pros::delay(1000);
    intakeR.retract();
    
    //pros::delay(1000);
    chassis.moveToPoint(-46,-0,1500,{.forwards = false, .maxSpeed = 127,.minSpeed = 60});
    chassis.waitUntilDone();
    BackC.retract();
    Intake.brake();
    chassis.moveToPoint(-46,-5,1500,{ .maxSpeed = 127,.minSpeed = 80});
    */
    /*chassis.turnToHeading(0,500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(24,-15,1500,{.forwards = false,.maxSpeed = 80,.minSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(500);
    
    BackC.extend();
    Intake.move(127);
    chassis.moveToPoint(20,-48,1500,{.maxSpeed = 127,.minSpeed = 100});
    pros::delay(750);
    chassis.moveToPoint(0,0,20000,{.maxSpeed = 127,.minSpeed = 100});
    currState = 5;*/
}

void redPosAuton(){
    //alliance no kiwami
    //currState = 5;
    chassis.setPose(-61, -8.92, 319.4);
    /*chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(-66,-4.108,750);
    chassis.waitUntilDone();
    currState = 4; 
    pros::delay(300);
    //intakeR.extend();
    chassis.moveToPoint(-61, -8.92, 750,{.forwards = false});
    chassis.waitUntilDone();
    currState=0;*/
    /*chassis.moveToPoint(-67,-4.108,1000);
    chassis.waitUntilDone();
    currState = 4; 
    pros::delay(1000);
    intakeR.extend();
    Intake.move(intakeVel);
    chassis.moveToPoint(-46.5, -1, 1000);
    pros::delay(600);
    intakeR.retract();
    */
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(-17,-24,1000,{.forwards = false,.maxSpeed = 80,.minSpeed = 40});
    //chassis.waitUntil(40);
    //Intake.brake();
    chassis.waitUntilDone();
    //currState=0;
    //
    BackC.extend();
    Intake.move(127);
    pros::delay(1000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    Intake.brake();
    chassis.turnToHeading(45, 500);
    /*chassis.moveToPoint(-25,-48,1000,{.forwards = true,.maxSpeed = 60,.minSpeed = 40});
    pros::delay(5000);
    currState = 5;
    chassis.moveToPoint(-24,0,1000,{.forwards = true,.maxSpeed = 60,.minSpeed = 40});*/
    chassis.moveToPoint(-16.5,-12,1500,{.forwards = true,.maxSpeed = 80,.minSpeed = 50,.earlyExitRange = 0.5});
    Intake.brake();
    chassis.turnToHeading(35, 750);
    chassis.waitUntilDone();
    doinkerR.extend();
    pros::delay(500);

    //kujuu no kiseki
    //chassis.swingToHeading(85, DriveSide::LEFT, 1500,{.direction = AngularDirection::CW_CLOCKWISE,.earlyExitRange = .5});
    //chassis.swingToHeading(70, DriveSide::RIGHT, 1500,{.direction = AngularDirection::CW_CLOCKWISE,.earlyExitRange = .5});
    chassis.turnToHeading(65, 1500);
    chassis.moveToPoint(-10, -12, 750);
    chassis.waitUntilDone();
    doinkerL.extend();
    
    pros::delay(500);
    chassis.waitUntilDone();
    Intake.move(-127);
    chassis.moveToPoint(-45,-45,1000,{.forwards = false,.maxSpeed = 100,.minSpeed = 80});
    chassis.waitUntilDone();
    doinkerR.retract();
    doinkerL.retract();
    chassis.swingToHeading(0, DriveSide::LEFT, 1500,{.earlyExitRange = 3});
    Intake.move(intakeVel);
    chassis.swingToHeading(200, DriveSide::RIGHT, 1500,{.direction = AngularDirection::CW_CLOCKWISE,.earlyExitRange = 3});
    doinkerL.extend();
    chassis.waitUntilDone();
    chassis.moveToPoint(-60,-60,1000,{.maxSpeed = 127,.minSpeed = 80});
    doinkerL.retract();
    pros::delay(2000);
    
    //chassis.moveToPoint(-50,-60,1000,{.forwards = false,.maxSpeed = 127,.minSpeed = 80}); 
    chassis.waitUntilDone();
    //BackC.retract();
    //chassis.moveToPoint(-50,-70,1000,{.maxSpeed = 127,.minSpeed = 80});
    chassis.turnToHeading(90, 500);
    Intake.brake();
    chassis.waitUntilDone();
    BackC.retract();
    pros::delay(1000);
    chassis.turnToHeading(270, 500);

    //chassis.waitUntilDone();
    //chassis.moveToPoint(-122,-75,2000,{.maxSpeed = 127,.minSpeed = 80});


}

void bluePosAuton(){
    americanPolice.set_led_pwm(100);
    //alliance no kiwami
    currState = 5;
    chassis.setPose(61, -8.92, 40.6);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(66,-4.108,750);
    chassis.waitUntilDone();
    currState = 4; 
    pros::delay(300);
    //intakeR.extend();
    chassis.moveToPoint(61, -8.92, 750,{.forwards = false});
    chassis.waitUntilDone();
    currState=0;
    Intake.move(intakeVel);
    /*
    chassis.moveToPoint(46, 2, 750);
    pros::delay(600);
    intakeR.retract();
    chassis.moveToPoint(46, -8, 750,{.forwards = false});
    pros::delay(750);
    chassis.moveToPoint(46, 0, 550);
    pros::delay(250);
    Intake.brake();*/
    chassis.moveToPoint(34,-24,1500,{.forwards = false,.maxSpeed = 80,.minSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(500);
    BackC.extend();
    Intake.move(intakeVel);
    /*chassis.moveToPoint(18,-18 ,1000,{.forwards = true,.maxSpeed = 127,.minSpeed = 50,.earlyExitRange = 1});
    chassis.turnToHeading(350, 500);
    //chassis.moveToPoint(5,-5,1000,{.forwards = true,.maxSpeed = 127,.minSpeed = 50,.earlyExitRange = 1});
    chassis.waitUntilDone();
    Intake.brake();
    doinkerL.extend();
    pros::delay(250);
    chassis.swingToHeading(300, DriveSide::LEFT, 1500,{.earlyExitRange = 3});
    chassis.moveToPoint(13,-13,1000,{.forwards = true,.maxSpeed = 127,.minSpeed = 50,.earlyExitRange = 1});
    chassis.waitUntilDone();
    doinkerR.extend();
    pros::delay(250);
    chassis.waitUntilDone();
    //Intake.move(-127);
    chassis.moveToPoint(45,-45,1000,{.forwards = false,.maxSpeed = 127,.minSpeed = 80});
    chassis.waitUntilDone();
    doinkerR.retract();
    doinkerL.retract();
    chassis.swingToHeading(0, DriveSide::RIGHT, 1500,{.maxSpeed=127,.minSpeed = 120,.earlyExitRange = 3});
    Intake.move(intakeVel);
    chassis.swingToHeading(135, DriveSide::LEFT, 1500,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=127,.minSpeed = 120,.earlyExitRange = 3});
    doinkerR.extend();
    chassis.waitUntilDone();*/
   //chassis.moveToPoint(40,-52,1000,{.maxSpeed = 127,.minSpeed = 80});
    chassis.moveToPoint(27,-58,1000,{.maxSpeed = 60,.minSpeed = 40});
    pros::delay(2000);
    Intake.brake();
    chassis.moveToPoint(72, -72, 1500);
    chassis.waitUntilDone();
    Intake.move(127);
    pros::delay(3000);
    chassis.moveToPoint(40, -40, 1500,{.forwards = false});
    currState = 5;
    pros::delay(1000);
    chassis.moveToPoint(12,-12,2000);
    //doinkerR.retract();
   /* chassis.moveToPoint(70,-70,1000,{.maxSpeed = 127,.minSpeed = 80});
    Intake.brake();
    chassis.waitUntilDone();
    doinkerR.extend();
    chassis.turnToHeading(0, 500);*/
    
    //chassis.moveToPoint(50,-60,1000,{.forwards = false,.maxSpeed = 127,.minSpeed = 80});    
    //chassis.waitUntilDone();
    //chassis.moveToPoint(-122,-75,2000,{.maxSpeed = 127,.minSpeed = 80});
    
}

void redPosAutonQual(){
    americanPolice.set_led_pwm(100);
    //alliance no kiwami
    currState = 5;
    chassis.setPose(-61, -8.92, 320);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(-66,-4.108,750);
    chassis.waitUntilDone();
    currState = 4; 
    pros::delay(300);
    //intakeR.extend();
    chassis.moveToPoint(-61, -8.92, 750,{.forwards = false});
    chassis.waitUntilDone();
    currState=0;
    Intake.move(intakeVel);
    /*
    chassis.moveToPoint(46, 2, 750);
    pros::delay(600);
    intakeR.retract();
    chassis.moveToPoint(46, -8, 750,{.forwards = false});
    pros::delay(750);
    chassis.moveToPoint(46, 0, 550);
    pros::delay(250);
    Intake.brake();*/
    chassis.moveToPoint(-34,-24,1500,{.forwards = false,.maxSpeed = 80,.minSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(500);
    BackC.extend();
    Intake.move(intakeVel);
    /*chassis.moveToPoint(18,-18 ,1000,{.forwards = true,.maxSpeed = 127,.minSpeed = 50,.earlyExitRange = 1});
    chassis.turnToHeading(350, 500);
    //chassis.moveToPoint(5,-5,1000,{.forwards = true,.maxSpeed = 127,.minSpeed = 50,.earlyExitRange = 1});
    chassis.waitUntilDone();
    Intake.brake();
    doinkerL.extend();
    pros::delay(250);
    chassis.swingToHeading(300, DriveSide::LEFT, 1500,{.earlyExitRange = 3});
    chassis.moveToPoint(13,-13,1000,{.forwards = true,.maxSpeed = 127,.minSpeed = 50,.earlyExitRange = 1});
    chassis.waitUntilDone();
    doinkerR.extend();
    pros::delay(250);
    chassis.waitUntilDone();
    //Intake.move(-127);
    chassis.moveToPoint(45,-45,1000,{.forwards = false,.maxSpeed = 127,.minSpeed = 80});
    chassis.waitUntilDone();
    doinkerR.retract();
    doinkerL.retract();
    chassis.swingToHeading(0, DriveSide::RIGHT, 1500,{.maxSpeed=127,.minSpeed = 120,.earlyExitRange = 3});
    Intake.move(intakeVel);
    chassis.swingToHeading(135, DriveSide::LEFT, 1500,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=127,.minSpeed = 120,.earlyExitRange = 3});
    doinkerR.extend();
    chassis.waitUntilDone();*/
   //chassis.moveToPoint(40,-52,1000,{.maxSpeed = 127,.minSpeed = 80});
    chassis.moveToPoint(-30,-58,1000,{.maxSpeed = 60,.minSpeed = 40});
    pros::delay(5000);
    Intake.brake();
    //chassis.moveToPoint(-72, -72, 1500);
    /*chassis.waitUntilDone();
    Intake.move(127);
    pros::delay(3000);
    chassis.moveToPoint(-40, -40, 1500,{.forwards = false});*/
    currState = 5;
    pros::delay(1000);
    chassis.moveToPoint(-12,-12,2000);
    //doinkerR.retract();
   /* chassis.moveToPoint(70,-70,1000,{.maxSpeed = 127,.minSpeed = 80});
    Intake.brake();
    chassis.waitUntilDone();
    doinkerR.extend();
    chassis.turnToHeading(0, 500);*/
    
    //chassis.moveToPoint(50,-60,1000,{.forwards = false,.maxSpeed = 127,.minSpeed = 80});    
    //chassis.waitUntilDone();
    //chassis.moveToPoint(-122,-75,2000,{.maxSpeed = 127,.minSpeed = 80});
    
}


void BlueNegativenottoday(){//Q143
    
    currState = 5;
    chassis.setPose(61, 8.92, 139.4);
    chassis.moveToPoint(66,4.108,1000);
    

    chassis.waitUntilDone();
    currState = 4; 
    pros::delay(1000);
    chassis.moveToPoint(61, 8.92, 750,{.forwards = false});
    chassis.moveToPoint(30,24,1000,{.forwards = false,.maxSpeed = 75,.minSpeed = 40});
    chassis.waitUntilDone();
    currState=0;
    pros::delay(250);
    BackC.extend();
    Intake.move(127);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(24,48,1750,{.maxSpeed = 60, .minSpeed=40});
    chassis.waitUntilDone();
    pros::delay(5000);
    currState=5;
    chassis.moveToPoint(24,0,1750,{.maxSpeed = 60, .minSpeed=40});
    /*chassis.waitUntilDone();
    intakeR.extend();
    chassis.moveToPoint(-48.981,-0,1750,{.maxSpeed = 60});
    chassis.waitUntilDone();
    intakeR.retract();
    pros::delay(500);
    chassis.moveToPoint(-48,15,1750,{.forwards = false});
    chassis.moveToPoint(-48,-10,1750,{.forwards = true});
    pros::delay(2000);*/
    chassis.moveToPoint(24,0,1750,{.forwards = true});
}

void redposGoal(){
    chassis.setPose(-62.712, -36.074, 90);
    Intake.move(127);
    chassis.moveToPoint(-35, -46, 1500,{.maxSpeed = 127,.minSpeed = 120});
    chassis.waitUntilDone();
    
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(-21, -42, 1500,{.maxSpeed = 80,.minSpeed = 40});
    pros::delay(350);
    Intake.brake();
    chassis.waitUntilDone();
    doinkerR.extend();
    pros::delay(300);
    chassis.moveToPoint(-48, -37, 1500,{.forwards= false,.maxSpeed = 127,.minSpeed = 120});
    chassis.waitUntilDone();
    doinkerR.retract();
    pros::delay(500);
    chassis.turnToHeading(340, 1500,{.direction = AngularDirection::CW_CLOCKWISE});
    chassis.moveToPoint(-49, -40, 1500,{.forwards= false,.maxSpeed = 80,.minSpeed = 40});
    chassis.waitUntilDone();
    BackC.extend();
    pros::delay(250);
    Intake.move(127);
    pros::delay(2000);
    Intake.brake();
    chassis.turnToHeading(0,500);
    chassis.waitUntilDone();
    BackC.retract();
    chassis.moveToPoint(-38, -16, 1500,{.forwards= false,.maxSpeed = 80,.minSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(250);
    BackC.extend();


    chassis.turnToHeading(270, 750);
    chassis.waitUntilDone();
    Intake.move(127);
    chassis.moveToPoint(-65, -12, 1500);
    pros::delay(2000);

    chassis.moveToPoint(-40, -25, 1500,{.maxSpeed = 127,.minSpeed = 120});

}

void blueposGoal(){
    chassis.setPose(62.712, -36.074, 90);
    Intake.move(127);
    chassis.moveToPoint(35, -46, 1500,{.maxSpeed = 127,.minSpeed = 120});
    chassis.waitUntilDone();
    
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.moveToPoint(21, -42, 1500,{.maxSpeed = 80,.minSpeed = 40});
    pros::delay(350);
    Intake.brake();
    chassis.waitUntilDone();
    doinkerR.extend();
    pros::delay(1000);
    pros::delay(300);
    chassis.moveToPoint(48, -37, 1500,{.forwards= false,.maxSpeed = 127,.minSpeed = 120});
    chassis.waitUntilDone();
    doinkerR.retract();
    pros::delay(500);
    chassis.turnToHeading(20, 1500,{.direction = AngularDirection::CW_CLOCKWISE});
    chassis.moveToPoint(45, -43, 1500,{.forwards= false,.maxSpeed = 80,.minSpeed = 40});
    
    BackC.extend();
    pros::delay(250);
    Intake.move(127);
    pros::delay(2000);
    Intake.brake();
    chassis.moveToPoint(24, -24, 1500,{.forwards= false,.maxSpeed = 80,.minSpeed = 40});
    BackC.extend();


    pros::delay(250);
    chassis.turnToHeading(90, 750);
    Intake.move(127);
    \
    chassis.waitUntilDone();
    Intake.move(127);
    chassis.moveToPoint(52, -24, 1500);


}
void autonomous(){

    
    pros::Task autonControlTask([]{
        while (true) {
            activeRacism();
            liftControl();
            pros::delay(10);
        }
    });
    pros::Task autonControlTask2([]{
        while (true) {
            antiStuck();
            pros::delay(10);
        }
    });

    
    redRace = 0;
    blueRace = 0;
    //redNegAuton();
    //blueNegAuton();
    redposGoal();
    //BlueNegativenottoday();
    //awpRedNeg();
    //awpBlueNeg();
    //redPosAuton();
    //bluePosAuton();
    //redPosAutonQual();
    /*chassis.setPose(0,0,0);
    chassis.moveToPose(0, 20, 0, 5000);*/
    //Mogo Rush
    /*
    if (auton == 1) {
        redNegAuton();
    }

    if (auton == 2) {
        awpRedNeg();
    }

    //Ring Side
    if (auton == 3) {
        bluePosAuton();
    }

    if (auton == 4){
        blueNegAuton();
    }

    if (auton == 5){
        awpBlueNeg();
    }


       
    */ 
}

/**
 * Runs in driver control
 */
void opcontrol() {
    bool backBool = 0;
    bool doinker = 0;
    bool IntakeP = 0;
    bool IntakeU = 0;
    currState = 0;
    //americanPolice.set_led_pwm(100);
    
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        if(controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            if(backBool){
                BackC.retract();
                backBool = 0;
            }else if(!backBool){
                BackC.extend();
                backBool = 1;

            }
        }
        if(controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            if(doinker){
                doinkerR.retract();
                //doinkerL.retract();
                doinker = 0;
            }else if(!doinker){
                doinkerR.extend();
                //doinkerL.extend();

                doinker = 1;

            }
        }
        if(controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            if(IntakeP){
                doinkerL.retract();
                IntakeP = 0;
            }else if(!IntakeP){
                doinkerL.extend();
                IntakeP = 1;

            }
        }
        if(controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            if(IntakeU){
                intakeR.retract();
                IntakeU = 0;
            }else if(!IntakeU){
                intakeR.extend();
                IntakeU = 1;

            }
        }
        //doinker A 
        if(controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            Intake.move(127);
        }else if (controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            Intake.move(-127);
        }else{
        Intake.brake();
        }
        
        if (controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			nextState();
		}
        if (controller1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			specialState();
		}


        // delay to save resources
        pros::delay(20);
    }
}