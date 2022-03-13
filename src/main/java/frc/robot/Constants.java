// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;

import com.fasterxml.jackson.databind.ser.std.NumberSerializers.IntLikeSerializer;

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

        ///////////////////
        // Driving ports //
        ///////////////////

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

        /////////////////
        // Pivot ports //
        /////////////////

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

        /**
         * The PWM port where the absolute position of the pivot motor is acquired.
         */
        public static final int FRONT_LEFT_ABSOLUTE_PWM_PORT = FRONT_LEFT + 0;

        /**
         * The PWM port where the absolute position of the pivot motor is acquired.
         */
        public static final int BACK_LEFT_ABSOLUTE_PWM_PORT = BACK_LEFT + 0;

        /**
         * The PWM port where the absolute position of the pivot motor is acquired.
         */
        public static final int BACK_RIGHT_ABSOLUTE_PWM_PORT = BACK_RIGHT + 0;

        /**
         * The PWM port where the absolute position of the pivot motor is acquired.
         */
        public static final int FRONT_RIGHT_ABSOLUTE_PWM_PORT = FRONT_RIGHT + 0;

        /**
         * When we're debugging, we need an easy way to abbreviate certain modules
         * and variables when they're in groups of four (one per pivot wheel).
         */
        public static final String[] CORNER_NAME_ABBREVS = {
                "FL", "BL", "BR", "FR"
        };

        /**
         * We need to make an assumption on where the Xbox controllers are plugged in.
         * This value may change if you plug other human input devices into the driver
         * station first.
         */
        public static int XBOX_CONTROLLER_PORT = 0;

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

        private static double WHEEL_BASE_DIAGONAL_METERS = Math.sqrt(WHEEL_BASE_WIDTH_METERS * WHEEL_BASE_WIDTH_METERS +
                        WHEEL_BASE_LENGTH_METERS * WHEEL_BASE_LENGTH_METERS);

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

        /** Deadzone value for the joystick channels of the controllers*/
        public static double JOYSTICK_DEAD_ZONE = 0.362;

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