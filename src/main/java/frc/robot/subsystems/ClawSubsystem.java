package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import frc.robot.util.GroundedDigitalInput;


public class ClawSubsystem extends SubsystemBase {
    /* ShuffleBoard */
	public static final ShuffleboardTab CLIMBER_TAB = Shuffleboard.getTab("Climber");

	public ClawSubsystem() 
    {
        	//ClawMotor
	private final CANSparkMax clawCanSparkMax = new CANSparkMax(19, MotorType.kBrushless);

	clawCanSparkMax.setInverted(true);
	clawCanSparkMax.setIdleMode(IdleMode.kBrake);
	private final RelativeEncoder clawEncoder = clawCanSparkMax.getEncoder();

	//limit switches
	public final GroundedDigitalInput outerLeft = new GroundedDigitalInput(6);
	public final GroundedDigitalInput outerRight = new GroundedDigitalInput(9);
	public final GroundedDigitalInput innerLeft = new GroundedDigitalInput(15);
	public final GroundedDigitalInput innerRight = new GroundedDigitalInput(22);
    

	//claw motor getter
	public CANSparkMax getClawCanSparkMax() {
		return clawCanSparkMax;
	}

	//limits
	public boolean outerLimitReached() {
		return outerLeft.get() || outerRight.get();
	}

	public boolean topLimitReached() {
		return innerRight.get() || innerLeft.get();
	}
    }


	@Override
	public void periodic() 
    {
	}
}

