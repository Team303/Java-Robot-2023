// Copyright (c) 2022 Team 303

package com.team303.robot;

import java.util.Arrays;
import java.util.List;

import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.team303.lib.math.DeadbandFilter;

import edu.wpi.first.wpilibj.util.Color;

/*
TODO Change around all the CAN IDs to fit
*/

public final class RobotMap {

	public static final class Swerve {

		/* CAN IDs of Drive Motors */
		public static final int LEFT_FRONT_DRIVE_ID = 2;
		public static final int LEFT_BACK_DRIVE_ID = 5;
		public static final int RIGHT_FRONT_DRIVE_ID = 8;
		public static final int RIGHT_BACK_DRIVE_ID = 11;

		/*CAN IDs of steer Motors*/
		public static final int LEFT_FRONT_STEER_ID = 1;
		public static final int LEFT_BACK_STEER_ID = 4;
		public static final int RIGHT_FRONT_STEER_ID = 7;
		public static final int RIGHT_BACK_STEER_ID = 10;

		/*Steer Encoder CAN IDs */
		public static final int LEFT_FRONT_STEER_CANCODER_ID = 3;
		public static final int LEFT_BACK_STEER_CANCODER_ID = 6;
		public static final int RIGHT_FRONT_STEER_CANCODER_ID = 9;
		public static final int RIGHT_BACK_STEER_CANCODER_ID = 12;

		/*Steer Motor Offset*/
		public static final double LEFT_FRONT_STEER_OFFSET= 0.0;
		public static final double RIGHT_FRONT_STEER_OFFSET = 0.0;
		public static final double LEFT_BACK_STEER_OFFSET = 0.0;
		public static final double RIGHT_BACK_STEER_OFFSET = 0.0;

		/* Drivebase Motor Inversion */
		public static final boolean LEFT_FRONT_SPARK_INVERTED = true;
		public static final boolean LEFT_BACK_SPARK_INVERTED = true;
		public static final boolean RIGHT_FRONT_SPARK_INVERTED = false;
		public static final boolean RIGHT_BACK_SPARK_INVERTED = false;

		/*Drive Train Dimentions*/
		public static final double TRACKWIDTH = 0.762;
		public static final double WHEELBASE = 0.762;
		public static final double ROTATION_CONSTANT = 2/Math.hypot(TRACKWIDTH, WHEELBASE);

		/* Motor Encoder Calculations */
		public static final double WHEEL_DIAMTER = 0.1524; // Diameter in meters
		public static final int ENCODER_COUNTS_PER_REV = 4096; // ctre CANCoder
		public static final double DRIVE_GEAR_RATIO = 12.75; // Toughbox mini 12.75:1
		public static final double DISTANCE_PER_ENCODER_PULSE; // Inches traveled for each encoder unit
		public static final double MAX_VELOCITY = 4.4196;
		public static final double MAX_ACCELERATION = 3; //Meters per second
		public static final double MAX_RPS = 183.33; // Max rotations per second

		/* Starting Position */
		public static final double STARTING_X = 0;
		public static final double STARTING_Y = 0;

		/* Default Module Configurations */
		public static final MechanicalConfiguration MK4I_L2_LEFT_FRONT = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    	);

		public static final MechanicalConfiguration MK4I_L2_LEFT_BACK = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    	);

		public static final MechanicalConfiguration MK4I_L2_RIGHT_FRONT = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    	);

		public static final MechanicalConfiguration MK4I_L2_RIGHT_BACK = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (14.0 / 50.0) * (10.0 / 60.0),
            false
    	);


		static {
			double wheelCircumference = WHEEL_DIAMTER * Math.PI;
			double motorRotationsPerEncoderPulse = 1 / ENCODER_COUNTS_PER_REV;
			double axelRotationsPerMotorRotation = 1 / MK4I_L2_LEFT_FRONT.getDriveReduction();

			DISTANCE_PER_ENCODER_PULSE = motorRotationsPerEncoderPulse
					* axelRotationsPerMotorRotation
					* wheelCircumference;
		}
	}

	public static final class Arm {

		public static final int SHOULDER_JOINT_ID = 17;
		public static final int ELBOW_JOINT_ID = 18;
		public static final int CLAW_JOINT_ID = 19;

		//TODO: Find claw constant values, confirm segment lengths
		public static final List<Double> SEGMENT_MASSES = Arrays.asList(10.323,8.722,0.0); //lbs.
		public static final List<Double> SEGMENT_LENGTHS_INCHES = Arrays.asList(36.0,36.0,12.0); //in.
		public static final List<Double> JOINT_TO_CENTER_OF_MASSES = Arrays.asList(7.138,9.806,0.0); //in.

		public static final float SOFT_LIMIT = 1000;

		public static final boolean CLIMB_MOTOR_INVERTED = true;
	}

	public static final class Auto {
		public static final double MAX_VELOCITY = 10;
		public static final double MAX_ACCELERATION = 10;
		public static final double RAMSETE_B = 10;
		public static final double RAMSETE_ZETA = 10;
	}
	
	public static final class PhotonvisionConstants {
		public static final double CAMERA_HEIGHT_METERS = 0; //NOT FINAL
		public static final double GRID_TARGET_HEIGHT_METERS = 0.36;
		public static final double DOUBLE_SUBSTATION_TARGET_HEIGHT_METERS = 0.59;
		public static final double CAMERA_PITCH_RADIANS = 0; //NOT FINAL
	}


	public static final class IOConstants {

		public static final int LEFT_JOYSTICK_ID = 1;
		public static final int RIGHT_JOYSTICK_ID = 2;
		public static final int OPERATOR_CONTROLLER = 0;

		public static final double DEADBAND_UPPERBOUND = 1;
		public static final double DEADBAND_LOWERBOUND = 0.05;
		public static final DeadbandFilter DEADBAND_FILTER = new DeadbandFilter(DEADBAND_LOWERBOUND, DEADBAND_UPPERBOUND);
	}

	public static final class LED {

		public static final int LED_ID = 1;
		public static final int BUFFER_LENGTH = 200;

		public static final Color RED = new Color(255 / 255D, 0, 0);
		public static final Color BLUE = new Color(0, 0, 255 / 255D);

		public static final Color FLASH_PRIMARY = new Color(
				255 / 255D,
				0 / 255D,
				0 / 255D);
		public static final Color FLASH_SECONDARY = new Color(
				230 / 255D,
				50 / 255D,
				0 / 255D);

		public static final Color TELEOP_COLOR = new Color(
				0 / 255D,
				255 / 255D,
				0 / 255D);
		public static final Color AUTONOMOUS_COLOR = new Color(
				230 / 255D,
				30 / 255D,
				0 / 255D);
		public static final Color DISABLED_COLOR = new Color(
				0 / 255D,
				90 / 255D,
				20 / 255D);
	}
}