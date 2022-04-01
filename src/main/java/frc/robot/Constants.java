// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        ///////////////////////
        // Common constants. //
        ///////////////////////

        /**
         * When we're debugging, we need an easy way to abbreviate certain modules
         * and variables when they're in groups of four (one per pivot wheel).
         */
        public static final String[] CORNER_NAME_ABBREVS = {
                "FL", "BL", "BR", "FR"
        };

        /**
         * Our swerve drive makes us use arrays of four quite often: four pivot motor
         * controllers, four driving motor controllers, and two sets of four PWM ports
         * for these. It helps to have a consistent order!
         *
         * These constants determine that order. You can think of them as being ordered
         * "counterclockwise around the car", starting with the "driver's seat."
         */
        public static final int FRONT_LEFT = 0;

        /**
         * @see #FRONT_LEFT
         */
        public static final int BACK_LEFT = 1;

        /**
         * @see #FRONT_LEFT
         */
        public static final int BACK_RIGHT = 2;

        /**
         * @see #FRONT_LEFT
         */
        public static final int FRONT_RIGHT = 3;

        /////////////////////////
        // Drive motor CAN IDs //
        /////////////////////////

        /**
         * The CAN bus ID for the motor controller which drives the front left wheel.
         */
        public static final int FRONT_LEFT_DRIVE_MOTOR_CAN_ID = FRONT_LEFT + 1;

        /**
         * The CAN bus ID for the motor controller which drives the back left wheel.
         */
        public static final int BACK_LEFT_DRIVE_MOTOR_CAN_ID = BACK_LEFT + 1;

        /**
         * The CAN bus ID for the motor controller which drives the back right wheel.
         */
        public static final int BACK_RIGHT_DRIVE_MOTOR_CAN_ID = BACK_RIGHT + 1;

        /**
         * The CAN bus ID for the motor controller which drives the front right wheel.
         */
        public static final int FRONT_RIGHT_DRIVE_MOTOR_CAN_ID = FRONT_RIGHT + 1;

        /////////////////////////
        // Pivot motor CAN IDs //
        /////////////////////////

        /**
         * The CAN bus ID for the motor controller which pivots the front left wheel.
         */
        public static final int FRONT_LEFT_TURN_MOTOR_CAN_ID = FRONT_LEFT + 5;

        /**
         * The CAN bus ID for the motor controller which pivots the back left wheel.
         */
        public static final int BACK_LEFT_TURN_MOTOR_CAN_ID = BACK_LEFT + 5;

        /**
         * The CAN bus ID for the motor controller which pivots the back right wheel.
         */
        public static final int BACK_RIGHT_TURN_MOTOR_CAN_ID = BACK_RIGHT + 5;

        /**
         * The CAN bus ID for the motor controller which pivots the front right wheel.
         */
        public static final int FRONT_RIGHT_TURN_MOTOR_CAN_ID = FRONT_RIGHT + 5;

        ///////////////////
        // Other CAN IDs //
        ///////////////////

        /**
         * We use a Spark MAX to control the NEO 550 that turns the shooter turntable.
         *
         * We need it to be on the CAN bus (as opposed to using PWM) because its position
         * must be carefully controlled to prevent overturning.
         */
        public static final int SHOOTER_TURNTABLE_CAN_ID = 9;

        /**
         * The turntable's 140-tooth gear connects to a 10-tooth gear at the end of the
         * shooter turntable motor's gearbox.  One rotation of the shooter turntable
         * motor therefore causes a 1/14th turn of the shooter turntable.
         *
         */
        public static final double SHOOTER_TURN_TABLE_GEAR_RATIO = 10.0/140.0;

        /**
         * Identify the indexer for both CAN Spark Maxes on the right and left side.
         */
        public static final int INDEXER_ROLLER_RIGHT_CAN_ID = 10;
        public static final int INDEXER_ROLLER_LEFT_CAN_ID = 11;

        /**
         * Identify the Flywheel for both CAN Spark Maxes on the right and left side.
         */
        public static final int FLYWHEEL_RIGHT_CAN_ID = 12;
        public static final int FLYWHEEL_LEFT_CAN_ID = 13;

        /**
         * Indentify the Climber for CAN Spark Maxes on the climber mechanism.
         */
        public static final int CLIMBER_CAN_ID = 14;

        /**
         * What speed value, from 0.0 to 1.0, to give the climber when it is climbing up
         * (pushing away from the ground) or climbing down (pulling toward the ground).
         */
        public static final double CLIMBER_SPEED = 0.40;

        ////////////////////////////
        // Controller (USB) ports //
        ////////////////////////////

        /**
         * Xbox contoller remote types are detected with both xbox controllers and regular
         * joystick controllers. By identifying the names of the pre-existing xbox controllers
         * we will be able to differentiate between a normal joystick and xbox controller.
         *
         * The normal joystick's name is "Logitech Extreme 3D"; please do NOT add it
         * to this list.
         */
        public static final String[] KNOWN_CONTROLLER_NAMES = {
                "Logitech Dual Action",              // We can't find this in the lab
                "Controller (Xbox One For Windows)", // An actual XBox controller
                "Pro Controller"                     // Nintendo Switch controller
        };

        public static final String[] DRIVE_CONTROLLER_NAME_PRIORITY = {
                // An acutal XBox controller. Driver preference and can be programmed to
                // rumble to distiguish between controller types.
                "Controller (Xbox One for Windows)",

                // Jacob's Black XBox Controller
                "Bluetooth XINPUT compatible input device"
        };

        public static final String[] AUXILARY_CONTROLLER_NAME_PRIORITY = {
                // Nintendo Switch controller. Same setup as an Xbox controller but second
                // priority to the drive controller.
                "Pro Controller"
        };

        /**
         * We need to make an assumption on where the Xbox controllers are plugged in.
         * This value may change if you plug other human input devices into the driver
         * station first.
         */
        public static int XBOX_CONTROLLER_PORT = 0;

        ///////////////
        // PWM ports //
        ///////////////

        public static final int INTAKE_MOTOR_PWM_PORT = 0; // 14
        public static final int UPTAKE_MOTOR_PWM_PORT = 1; // 5
        public static final int HOOD_LEFT_SERVO_PWM_PORT = 2;
        public static final int HOOD_RIGHT_SERVO_PWM_PORT = 3;
        public static final int SHOOTER_UPPER_FLYWHEEL_PWM_PORT = 4; // Lower flywheel uses CAN.

        ////////////////
        // DIO ports. //
        ////////////////

        /**
         * The DIO port where the absolute position of the FL pivot motor is acquired
         * as a PWM value.
         */
        public static final int FRONT_LEFT_ABSOLUTE_DIO_PORT = FRONT_LEFT + 0;

        /**
         * The DIO port where the absolute position of the BL pivot motor is acquired
         * as a PWM value.
         */
        public static final int BACK_LEFT_ABSOLUTE_DIO_PORT = BACK_LEFT + 0;

        /**
         * The DIO port where the absolute position of the BR pivot motor is acquired
         * as a PWM value.
         */
        public static final int BACK_RIGHT_ABSOLUTE_DIO_PORT = BACK_RIGHT + 0;

        /**
         * The DIO port where the absolute position of the FR pivot motor is acquired.
         * as a PWM value.
         */
        public static final int FRONT_RIGHT_ABSOLUTE_DIO_PORT = FRONT_RIGHT + 0;

        /**
         * When the shooter hits this limit switch, it must abandon its current
         * direction of travel and only allow travel in the opposite direction....until,
         * that is, the limit switch is it again from the other side.
         */
        public static final int SHOOTER_TURN_TABLE_LIMIT_SWITCH_DIO_PORT = 4;


        /**
         * DIO port where the indexer sensor for the intake/uptake system is.
         * Indexer sensor tests when there is a ball in the indexer.
         */
        public static final int INDEX_SENSOR_DIO_PORT = 5;

        /**
         * Another break-beam infrared digital on/off sensor, this time for detecting
         * balls in the uptake.
         */
        public static final int UPTAKE_SENSOR_DIO_PORT = 6;

        /////////////////////////////////////////////
        // Constants measured in real-world units. //
        /////////////////////////////////////////////

        /**
         * Conversion factor for Feet to Meters
         */
        public static double FEET_TO_METERS = 0.3048;

        /**
         * This value is an assumed maximum speed of the robot in meters per second,
         * which we will use for calculations
         * Once the robot is built we will measure and update said speed so that it is
         * more accurate
         */
        public static double ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND = 10 * FEET_TO_METERS;

        /**
         * Width of the wheel base in meters from the center of one wheel on the left to
         * the center of one wheel on the right
         */
        public static double WHEEL_BASE_WIDTH_METERS = 22.5 /* inches */ / 12 * FEET_TO_METERS;

        /**
         * Length of the wheel base in meters from the center of one wheel on the back
         * to the center of one wheel on the front
         */
        public static double WHEEL_BASE_LENGTH_METERS = 22.5 /* inches */ / 12 * FEET_TO_METERS;

        /**
         * Length of the diagonal of the wheel base.  Needed to get the diameter of the
         * wheel base's circumcircle for a later calculation.
         */
        private static double WHEEL_BASE_DIAGONAL_METERS = Math.hypot(WHEEL_BASE_WIDTH_METERS,
                                                                      WHEEL_BASE_LENGTH_METERS);

        /**
         * Ratio for how many revolutions of the pivot motor correspond to one full
         * rotation of the wheel about the y-axis.
         *
         * The only way to get this value is to dismantle the swerve module and count
         * the teeth. So, we got this value from the tech specs at
         * https://www.swervedrivespecialties.com/products/mk3-swerve-module.
         */
        public static final double WHEEL_TURN_RATIO = 12.8 * (1.0);

        /**
         * Robot's maximum rotational velocity in radians per second
         * Value is guessed and needs to be measured
         *
         * Formula is derived from the robots linear speed divided by the circumference
         * of the circumscribed circle created by the wheel base rectangle
         */
        public static double ROBOT_MAXIMUM_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (2 * Math.PI)
                        * (ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND / (Math.PI * WHEEL_BASE_DIAGONAL_METERS));

        /** Deadzone value for the joystick channels of the controllers */
        public static double JOYSTICK_DEAD_ZONE = 0.362;

        /**
         * The exponent to use for the exponential curve when we adjust the values
         * coming from the joystick.  The drivers prefer the exponential response
         * because it offers finer-grained control when joystick values are lower.
         *
         * Use 1.0 to force a linear response curve.
         */
        public static final double JOYSTICK_RESPONSE_CURVE_EXPONENT = 2.0;

        /**
         * Distance from the ground to the center of the reflective tape on the
         * hoop. This value is the average of the distance from the top of the
         * tape and the bottom of the tape.
         */
        public static final double VISION_TARGET_AVERAGE_HEIGHT_INCHES = 102.6875;

        /**
         * Distance from the ground of the robot to the center of the lense on
         * robot. This is the best estimate for the mount angle of the Limelight
         * is 40 degrees.
         */
        public static final int LIMELIGHT_LENS_HEIGHT_INCHES = 25;

        public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 40;

        public static final double DEGREES_TO_RADIANS = Math.PI / 180;

        /** When declaring an alternate encoder we need to know the clicks per revolution. */
        public static int SRX_MAG_ENCODER_CLICKS_PER_REVOLUTION = 4096;

        /** Displacement of module from actual zero, used to reset them to zero relative to the whole robot. */
        public static double[] DISPLACEMENT_ANGLES = {
                166.0, // Front Left
                17.8, // Back Left
                255.0, // Back Right
                202.0  // Front Right
        };

        //////////////////////////
        // Compressor Constants //
        //////////////////////////

        public static final double MINIMUM_COMPRESSOR_PSI = 100;
        public static final double MAXIMUM_COMPRESSOR_PSI = 120;

        ////////////////////////////////////
        /// Shooter and Indexer Constants //
        ////////////////////////////////////

        /**
         * We desire for balls launched by the shooter to travel in a parabolic arc.  This is determined
         * by the lower flywheel(s) and hood, but we added the upper flywheel to perform fine adjustments
         * of the trajectory and mitigate backspin.
         *
         * The idea here is that we will reduce the number of variables we have to work with by always
         * making the upper flywheel speed a function of the lower flywheel speed.
         */
        public static final double UPPER_FLYWHEEL_MUTIPLIER = 2;

        /**
         * The speed at which to run the indexer motors when they are releasing a ball
         * to the shooter.
         */
        public static final double INDEXER_SPEED = -1.0;
}