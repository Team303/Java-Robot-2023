// Copyright (c) 2022 Team 303

package com.team303.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.team303.robot.Robot;
import com.team303.robot.RobotMap.Swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.RelativeEncoder;

public class SwerveSubsystem extends SubsystemBase {

	/* odometry sim */
	private final double SECOND_TO_MS = 1000.0;
	private final Timer timer = new Timer();
	private double timeElapsed = 0;
	private double lastPeriodic = 0;

	// private Rotation2d angle = new Rotation2d();
	private double angle = 0;
	private double positions[] = { 0, 0, 0, 0 };

	/* Field Sim */
	public static final Field2d field = new Field2d();

	/* ShuffleBoard */
	public static final ShuffleboardTab DRIVEBASE_TAB = Shuffleboard.getTab("Drivebase");
	// public static final ShuffleboardTab FIELD_TAB =

	public static final NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable("swerve");

	/* Swerve Modules */
	private final SwerveModule leftFrontModule;
	private final SwerveModule rightFrontModule;
	private final SwerveModule leftBackModule;
	private final SwerveModule rightBackModule;

	/* Chasis Speeds */
	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	/* Kinematics */
	private final SwerveDriveKinematics kinematics;

	/* Odometry */
	SwerveDriveOdometry odometry;

	private Pose2d pose = new Pose2d();
	private SwerveDrivePoseEstimator poseEstimator;

	public static final double MAX_VOLTAGE = 12.0;
			
	public static final double MAX_VELOCITY_METERS_PER_SECOND = Swerve.MAX_VELOCITY;

	public SwerveSubsystem() {

		timer.start();

		SmartDashboard.putData("FIELD SIM", field);

		kinematics = new SwerveDriveKinematics(
				new Translation2d(Swerve.TRACKWIDTH / 2.0, Swerve.WHEELBASE / 2.0),
				new Translation2d(Swerve.TRACKWIDTH / 2.0, -Swerve.WHEELBASE / 2.0),
				new Translation2d(-Swerve.TRACKWIDTH / 2.0, Swerve.WHEELBASE / 2.0),
				new Translation2d(-Swerve.TRACKWIDTH / 2.0, -Swerve.WHEELBASE / 2.0));

		leftFrontModule = new MkSwerveModuleBuilder()
				.withLayout(
						DRIVEBASE_TAB.getLayout("leftFrontModule", BuiltInLayouts.kGrid)
								.withSize(2, 4)
								.withPosition(4, 0))
				.withGearRatio(Swerve.MK4I_L2_LEFT_FRONT)
				.withDriveMotor(MotorType.NEO, Swerve.LEFT_FRONT_DRIVE_ID)
				.withSteerMotor(MotorType.NEO, Swerve.LEFT_FRONT_STEER_ID)
				.withSteerEncoderPort(Swerve.LEFT_FRONT_STEER_CANCODER_ID)
				.build();

		leftBackModule = new MkSwerveModuleBuilder()
				.withLayout(
						DRIVEBASE_TAB.getLayout("leftBackModule", BuiltInLayouts.kGrid)
								.withSize(2, 4)
								.withPosition(4, 0))
				.withGearRatio(Swerve.MK4I_L2_LEFT_BACK)
				.withDriveMotor(MotorType.NEO, Swerve.LEFT_BACK_DRIVE_ID)
				.withSteerMotor(MotorType.NEO, Swerve.LEFT_BACK_STEER_ID)
				.withSteerEncoderPort(Swerve.LEFT_BACK_STEER_CANCODER_ID)
				.build();

		rightFrontModule = new MkSwerveModuleBuilder()
				.withLayout(
						DRIVEBASE_TAB.getLayout("rightFrontModule", BuiltInLayouts.kGrid)
								.withSize(2, 4)
								.withPosition(4, 0))
				.withGearRatio(Swerve.MK4I_L2_RIGHT_FRONT)
				.withDriveMotor(MotorType.NEO, Swerve.RIGHT_FRONT_DRIVE_ID)
				.withSteerMotor(MotorType.NEO, Swerve.RIGHT_FRONT_STEER_ID)
				.withSteerEncoderPort(Swerve.RIGHT_FRONT_STEER_CANCODER_ID)
				.build();

		rightBackModule = new MkSwerveModuleBuilder()
				.withLayout(
						DRIVEBASE_TAB.getLayout("rightBackModule", BuiltInLayouts.kGrid)
								.withSize(2, 4)
								.withPosition(4, 0))
				.withGearRatio(Swerve.MK4I_L2_RIGHT_BACK)
				.withDriveMotor(MotorType.NEO, Swerve.RIGHT_BACK_DRIVE_ID)
				.withSteerMotor(MotorType.NEO, Swerve.RIGHT_BACK_STEER_ID)
				.withSteerEncoderPort(Swerve.RIGHT_BACK_STEER_CANCODER_ID)
				.build();

		if (Robot.isReal()) {
			odometry = new SwerveDriveOdometry(
					kinematics, Rotation2d.fromDegrees(Robot.navX.getAngle()),
					new SwerveModulePosition[] {
							leftFrontModule.getPosition(),
							leftBackModule.getPosition(),
							rightFrontModule.getPosition(),
							rightBackModule.getPosition(),
					}, new Pose2d(Swerve.STARTING_X, Swerve.STARTING_Y, new Rotation2d()));
		} else {
			odometry = new SwerveDriveOdometry(
				kinematics, Rotation2d.fromDegrees(0),
				new SwerveModulePosition[] {
						new SwerveModulePosition(0, new Rotation2d()),
						new SwerveModulePosition(0, new Rotation2d()),
						new SwerveModulePosition(0, new Rotation2d()),
						new SwerveModulePosition(0, new Rotation2d())
				}, new Pose2d(Swerve.STARTING_X, Swerve.STARTING_Y, new Rotation2d()));
		}

		poseEstimator = new SwerveDrivePoseEstimator(
				kinematics,
				new Rotation2d(),
				new SwerveModulePosition[] {
						new SwerveModulePosition(0.0, new Rotation2d()),
						new SwerveModulePosition(0.0, new Rotation2d()),
						new SwerveModulePosition(0.0, new Rotation2d()),
						new SwerveModulePosition(0.0, new Rotation2d()),
				},
				new Pose2d());
	}

	// return kinematics instance
	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	// return current positition and angle

	public Pose2d getPose() {
		return pose;
	}

	public SwerveDriveOdometry getOdometry() {
		return odometry;
	}

	public void resetOdometry() {
		odometry.resetPosition(Robot.navX.getRotation2d(), getModulePositions(), new Pose2d());
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(Robot.navX.getRotation2d(), getModulePositions(), pose);

	}

	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				leftFrontModule.getPosition(),
				leftBackModule.getPosition(),
				rightFrontModule.getPosition(),
				rightBackModule.getPosition(),
		};
	}

	public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
		rotation *= Swerve.ROTATION_CONSTANT;

		/* 
		if (DriverStation.getAlliance() == Alliance.Blue && Robot.isReal()) {
			translation = new Translation2d(-translation.getX(), -translation.getY());
		}

		if (translation.getNorm() <= Units.inchesToMeters(1.0 / 60)) {
			translation = new Translation2d();
		}

		if ( Math.abs(rotation) <= 1) {
			rotation = 0.0;
		}*/

		if (fieldOriented && Robot.isReal()) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
					Rotation2d.fromDegrees(Robot.navX.getAngle()));
		} else if (fieldOriented && !Robot.isReal()) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
					Rotation2d.fromRadians(angle));
		} else {
			chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
		}

		//drive(kinematics.toSwerveModuleStates(chassisSpeeds));
	}

	public void offCenterDrive(Translation2d translation, double rotation, Translation2d centerOfRotation,
			boolean fieldOriented) {
		rotation *= Swerve.ROTATION_CONSTANT;

		/* 

		if (DriverStation.getAlliance() == Alliance.Blue) {
			translation = new Translation2d(-translation.getX(), -translation.getY());
		}

		if (DriverStation.getAlliance() == Alliance.Blue && Robot.isReal()) {
			translation = new Translation2d(-translation.getX(), -translation.getY());
		}

		if (translation.getNorm() <= Units.inchesToMeters(1.0 / 60)) {
			translation = new Translation2d();
		}
		
		if ( Math.abs(rotation) <= 1) {
			rotation = 0.0;
		} */

		if (fieldOriented) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
					Rotation2d.fromDegrees(Robot.navX.getAngle()));
		} else {
			chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
		}

		//drive(kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation));
	}

	// generic drive method

	public void drive(SwerveModuleState[] state) {
		chassisSpeeds = kinematics.toChassisSpeeds(state);
		// map speed of swerve modules to voltage
	}

	public void stop() {
		leftFrontModule.set(0, leftFrontModule.getSteerAngle());
		rightFrontModule.set(0, rightBackModule.getSteerAngle());
		leftBackModule.set(0, leftBackModule.getSteerAngle());
		rightBackModule.set(0, rightFrontModule.getSteerAngle());
	}
	// Assuming this method is part of a drivetrain subsystem that provides the
	// necessary methods

	@Override
	public void periodic() {

		SwerveModuleState[] state = kinematics.toSwerveModuleStates(chassisSpeeds);

		leftFrontModule.set(state[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				state[0].angle.getRadians());
		rightFrontModule.set(state[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				state[1].angle.getRadians());
		leftBackModule.set(state[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				state[2].angle.getRadians());
		rightBackModule.set(state[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				state[3].angle.getRadians());

		if (Robot.isReal()) {
			// Update Pose
			pose = odometry.update(
					Rotation2d.fromDegrees(Robot.navX.getAngle()),
					new SwerveModulePosition[] {
							leftFrontModule.getPosition(),
							leftBackModule.getPosition(),
							rightFrontModule.getPosition(),
							rightBackModule.getPosition(),
					});
		} else {
			timeElapsed = (timer.get() - lastPeriodic) * SECOND_TO_MS;
			angle += chassisSpeeds.omegaRadiansPerSecond / SECOND_TO_MS * timeElapsed;
			positions[0] += state[0].speedMetersPerSecond / SECOND_TO_MS * timeElapsed;
			positions[1] += state[1].speedMetersPerSecond / SECOND_TO_MS * timeElapsed;
			positions[2] += state[2].speedMetersPerSecond / SECOND_TO_MS * timeElapsed;
			positions[3] += state[3].speedMetersPerSecond / SECOND_TO_MS * timeElapsed;

			pose = odometry.update(Rotation2d.fromRadians(angle),
					new SwerveModulePosition[] {
							new SwerveModulePosition(positions[0], state[0].angle),
							new SwerveModulePosition(positions[1], state[1].angle),
							new SwerveModulePosition(positions[2], state[2].angle),
							new SwerveModulePosition(positions[3], state[3].angle),
					});
		}

		lastPeriodic = timer.get();

		// field.setRobotPose(odometry.getPoseMeters());
		Logger.getInstance().recordOutput("Swerve Module States", kinematics.toSwerveModuleStates(chassisSpeeds));
		Logger.getInstance().recordOutput("Odometry", pose);
	}
}
