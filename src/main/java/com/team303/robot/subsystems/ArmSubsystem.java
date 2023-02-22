// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team303.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.team303.lib.kinematics.EffectorPathPlanner;
import com.team303.lib.kinematics.FabrikController;
import com.team303.lib.kinematics.RMACProfile;
import com.team303.robot.RobotMap.Arm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

	public static final ShuffleboardTab ARM_TAB = Shuffleboard.getTab("Arm");
	public static final NetworkTable armNetwork = NetworkTableInstance.getDefault().getTable("arm");
	public static final DoubleArraySubscriber JOINT_ANGLES_SUB = armNetwork.getDoubleArrayTopic("ja")
			.subscribe(new double[] { 0.0, 0.0, 0.0 });
	public static final DoubleArraySubscriber JOINT_RPM_SUB = armNetwork.getDoubleArrayTopic("jr")
			.subscribe(new double[] { 0.0, 0.0, 0.0 });

	public static final DoubleArrayPublisher JOINT_ANGLES_PUB = armNetwork.getDoubleArrayTopic("Joint Angles Out")
			.publish();
	public static final DoubleArrayPublisher JOINT_RPM_PUB = armNetwork.getDoubleArrayTopic("Joints RPM Out").publish();

	public class ShoulderJoint {
		private final CANSparkMax shoulderMotor = new CANSparkMax(Arm.SHOULDER_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder();
		public final PIDController shoulderFeedback = new PIDController(0.01, 0.0, 0.0);
		private MechanismLigament2d shoulderSimulator;

	}

	public class ElbowJoint {
		private final CANSparkMax elbowMotor = new CANSparkMax(Arm.ELBOW_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder elbowEncoder = elbowMotor.getEncoder();
		public final PIDController elbowFeedback = new PIDController(0.01, 0.0, 0.0);
		private MechanismLigament2d elbowSimulator;
	}

	public class ClawJoint {
		private final CANSparkMax clawMotor = new CANSparkMax(Arm.CLAW_JOINT_ID, MotorType.kBrushless);
		private final RelativeEncoder clawEncoder = clawMotor.getEncoder();
		public final PIDController clawFeedback = new PIDController(0.01, 0.0, 0.0);
		private MechanismLigament2d clawSimulator;
	}

	private static FabrikController armKinematics = new FabrikController();

	private ShoulderJoint shoulderJoint = new ShoulderJoint();
	private ElbowJoint elbowJoint = new ElbowJoint();
	private ClawJoint clawJoint = new ClawJoint();

	Mechanism2d armSimulation;

	public ArmSubsystem() {
		// Initialize Inverse Kinematics with constant values
		armKinematics.setArmLength(82f);
		armKinematics.setSegmentLengthRatio(0, 35 / 82f);
		armKinematics.setSegmentLengthRatio(1, 35 / 82f);
		armKinematics.setSegmentLengthRatio(2, 12 / 82f);
		armKinematics.setSegmentLengths();
		armKinematics.setAngleConstraint(0, 45, 45);
		armKinematics.setAngleConstraint(1, 135, 135);
		armKinematics.setAngleConstraint(2, 135, 135);
		armKinematics.setSegmentInitialDirection(0, (float) Math.PI / 2);
		armKinematics.setSegmentInitialDirection(1, 0f);
		armKinematics.setSegmentInitialDirection(2, (float) -Math.PI / 4);
		armKinematics.initializeArm();
		armKinematics.setSolveDistanceThreshold(5f);
		armKinematics.setMaxIterationAttempts(500);

		// TODO: Find joint gear ratios
		shoulderJoint.shoulderEncoder.setPositionConversionFactor(1);
		// 60 motor rotations = 360 degrees of rotation for the arm
		elbowJoint.elbowEncoder.setPositionConversionFactor(60);
		clawJoint.clawEncoder.setPositionConversionFactor(1);

		armSimulation = new Mechanism2d(168/Math.sqrt(2), 168/Math.sqrt(2));
		MechanismRoot2d armRoot = armSimulation.getRoot("shoulderJoint", 84/Math.sqrt(2), 84/Math.sqrt(2));
		shoulderJoint.shoulderSimulator = armRoot.append(new MechanismLigament2d("shoulder",
				(double) armKinematics.getSegmentLength(2), 0.0, 5.0, new Color8Bit(255, 0, 0)));
		elbowJoint.elbowSimulator = shoulderJoint.shoulderSimulator.append(new MechanismLigament2d("elbow",
				(double) armKinematics.getSegmentLength(1), 0.0, 5.0, new Color8Bit(0, 255, 0)));
		clawJoint.clawSimulator = elbowJoint.elbowSimulator.append(new MechanismLigament2d("claw",
				(double) armKinematics.getSegmentLength(0), 0.0, 5.0, new Color8Bit(0, 0, 255)));

	}

	public static NetworkTable getArmNetwork() {
		if (armNetwork == null) {
			return NetworkTableInstance.getDefault().getTable("arm");
		}
		return armNetwork;
	}

	public void reach(Translation3d translation) {
		List<Float> currentArmPosition = armKinematics.getEffectorPoint();
		armKinematics.solveTargetIK(translation);
		List<Float> desiredArmPosition = armKinematics.getEffectorPoint();
		RMACProfile RMACProfiledArmFeedForward = new RMACProfile(armKinematics,
				new RMACProfile.Properties(Arm.SEGMENT_MASSES, Arm.JOINT_TO_CENTER_OF_MASSES,
						Arm.SEGMENT_LENGTHS_INCHES),
				new EffectorPathPlanner(currentArmPosition, desiredArmPosition, 0.5f));
		final List<List<Double>> voltsList = RMACProfiledArmFeedForward.getFinalVolts();
		final List<List<Double>> posList = RMACProfiledArmFeedForward.getFinalJointPositions();
		for (int i = 0; i < voltsList.get(0).size() - 1; i++) {
			boolean shoulderSetpointInvert = getEncoderPosition()[0] - posList.get(i).get(0) > 0 ? true : false;
			boolean elbowSetpointInvert = getEncoderPosition()[1] - posList.get(i).get(1) > 0 ? true : false;
			boolean clawSetpointInvert = getEncoderPosition()[2] - posList.get(i).get(2) > 0 ? true : false;
			double shoulderEncoderPosition = getEncoderPosition()[0];
			double shoulderTargetPosition = posList.get(i).get(0);
			double elbowEncoderPosition = getEncoderPosition()[1];
			double elbowTargetPosition = posList.get(i).get(1);
			double clawEncoderPosition = getEncoderPosition()[2];
			double clawTargetPosition = posList.get(i).get(2);
			if (shoulderSetpointInvert) {
				shoulderEncoderPosition = -getEncoderPosition()[0];
				shoulderTargetPosition = -posList.get(i).get(0);
			}
			if (elbowSetpointInvert) {
				elbowEncoderPosition = -getEncoderPosition()[1];
				elbowTargetPosition = -posList.get(i).get(1);
			}
			if (clawSetpointInvert) {
				clawEncoderPosition = -getEncoderPosition()[2];
				clawTargetPosition = -posList.get(i).get(2);
			}
			while (shoulderEncoderPosition < shoulderTargetPosition || elbowEncoderPosition < elbowTargetPosition
					|| clawEncoderPosition < clawTargetPosition) {
				shoulderJoint.shoulderMotor.setVoltage(voltsList.get(0).get(i));
				elbowJoint.elbowMotor.setVoltage(voltsList.get(1).get(i));
				clawJoint.clawMotor.setVoltage(voltsList.get(2).get(i));
				if (shoulderSetpointInvert) {
					shoulderEncoderPosition = -getEncoderPosition()[0];
				}
				if (elbowSetpointInvert) {
					elbowEncoderPosition = -getEncoderPosition()[1];
				}
				if (clawSetpointInvert) {
					clawEncoderPosition = -getEncoderPosition()[2];
				}
			}
			shoulderJoint.shoulderMotor
					.setVoltage(voltsList.get(voltsList.size() - 1).get(0) + shoulderJoint.shoulderFeedback
							.calculate(getEncoderPosition()[0], posList.get(posList.size() - 1).get(0)));
			elbowJoint.elbowMotor.setVoltage(voltsList.get(voltsList.size() - 1).get(1) + elbowJoint.elbowFeedback
					.calculate(getEncoderPosition()[1], posList.get(posList.size() - 1).get(1)));
			clawJoint.clawMotor.setVoltage(voltsList.get(voltsList.size() - 1).get(2) + clawJoint.clawFeedback
					.calculate(getEncoderPosition()[2], posList.get(posList.size() - 1).get(2)));
		}
	}

	public void reach(List<Double> desiredDegreeAngles) {
		Matrix<N2, N1> forwardKinematics = new Matrix<N2,N1>(Nat.N2(), Nat.N1());
		for (int i = 0; i < desiredDegreeAngles.size(); i++) {
			Vector<N2> vectorDirection = VecBuilder.fill(Math.cos(desiredDegreeAngles.get(i)),
					Math.sin(desiredDegreeAngles.get(i)));
			forwardKinematics = forwardKinematics.plus(vectorDirection.times(armKinematics.getSegmentLength(i)));
		}
		reach(new Translation3d(forwardKinematics.get(0, 0), 0.0, forwardKinematics.get(1, 0)));
	}

	public void resetEncoders() {
		shoulderJoint.shoulderEncoder.setPosition(0.0);
		elbowJoint.elbowEncoder.setPosition(0.0);
		clawJoint.clawEncoder.setPosition(0.0);
	}

	public double[] getEncoderPosition() {
		return new double[] { shoulderJoint.shoulderEncoder.getPosition(), elbowJoint.elbowEncoder.getPosition(),
				clawJoint.clawEncoder.getPosition() };
	}

	public double[] getJointResolutions() {
		return new double[] { shoulderJoint.shoulderEncoder.getCountsPerRevolution(),
				elbowJoint.elbowEncoder.getCountsPerRevolution(), clawJoint.clawEncoder.getCountsPerRevolution() };
	}

	@Override
	public void periodic() {
		JOINT_ANGLES_PUB.set(JOINT_ANGLES_SUB.get(new double[] { 0.0, 0.0, 0.0 }));
		JOINT_RPM_PUB.set(JOINT_RPM_SUB.get(new double[] { 0.0, 0.0, 0.0 }));
		Logger.getInstance().recordOutput("MyMechanism", this.armSimulation);
	}
}
