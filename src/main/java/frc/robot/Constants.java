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

        /**
         * We need to make an assumption on where the Xbox controllers are plugged in.
         * This value may change if you plug other human input devices into the driver
         * station first.
         */
        public static int XBOX_CONTROLLER_PORT = 0;

        ///////////////
        // PWM ports //
        ///////////////

        public static final int INTAKE_MOTOR_PWM_PORT = 0;
        public static final int UPTAKE_MOTOR_PWM_PORT = 1;
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

        /** When declaring an alternate encoder we need to know the clicks per revolution. */
        public static int SRX_MAG_ENCODER_CLICKS_PER_REVOLUTION = 4096;

        /** Displacement of module from actual zero, used to reset them to zero relative to the whole robot. */
        public static double[] DISPLACEMENT_ANGLES = {
                166.0, // Front Left
                17.8, // Back Left
                255.0, // Back Right
                202.0  // Front Right
        };
}