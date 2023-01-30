package com.team303.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.team303.robot.util.GroundedDigitalInput;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
	/* ShuffleBoard */
	public static final ShuffleboardTab CLIMBER_TAB = Shuffleboard.getTab("Climber");

	private final CANSparkMax clawCanSparkMax = new CANSparkMax(20, MotorType.kBrushless);
	private final CANSparkMax rotateCANSparkMax = new CANSparkMax(21, MotorType.kBrushless);
	private final RelativeEncoder clawEncoder;
	private final RelativeEncoder rotateEncoder;
	public final GroundedDigitalInput outerLeft;
	public final GroundedDigitalInput outerRight;
	public final GroundedDigitalInput innerLeft;
	public final GroundedDigitalInput innerRight;

	public ClawSubsystem() {
		clawCanSparkMax.setInverted(true);
		clawCanSparkMax.setIdleMode(IdleMode.kBrake);
		clawEncoder = clawCanSparkMax.getEncoder();
		rotateEncoder = rotateCANSparkMax.getEncoder();

		// limit switches
		outerLeft = new GroundedDigitalInput(0);
		outerRight = new GroundedDigitalInput(1);
		innerLeft = new GroundedDigitalInput(2);
		innerRight = new GroundedDigitalInput(3);
    }

	//claw motor getter among us imposter mode
	public CANSparkMax getClawCanSparkMax() {
		return clawCanSparkMax;
	}

	//limits
	public boolean outerLimitReached() {
		return outerLeft.get() || outerRight.get();
	}
	
	public boolean innerLimitReached() {
		return innerRight.get() || innerLeft.get();
	}

	public void claw(double speed) {
		if ((outerLimitReached() && speed < 0) || (innerLimitReached() && speed > 0)) 
			{
			clawCanSparkMax.set(0);
			return;
		}
		clawCanSparkMax.set(speed);
    }

	public void rotate(double speed) {
		rotateCANSparkMax.set(speed);
	}
        
	public boolean getEncoderPos(double encoderLimit) {
		return clawEncoder.getPosition() == encoderLimit;
	}

	public boolean getRotate(double rotateLimit) {
		return rotateEncoder.getPosition() == rotateLimit;
	}

	public void resetEncoders() {
		clawEncoder.setPosition(0.0);
		rotateEncoder.setPosition(0.0);
	}
}