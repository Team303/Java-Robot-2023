package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;
import static com.team303.robot.Robot.limelight;
import static com.team303.robot.Robot.swerve;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

//Make sure to run CrosshairAlign before this command!
public class ReachConeToNodeCommand extends CommandBase {

    public static PIDController xControl;

    List<Double> desiredAngles = new ArrayList<Double>() {
        {
            add(0.0);
            add(90.0);
            add(90.0);
        }
    }; 

    public ReachConeToNodeCommand() {
        addRequirements(swerve, arm);
        xControl = new PIDController(0.01, 0, 0);
    }

    @Override
    public void execute() {
        swerve.drive(
                new Translation2d(
                        xControl.calculate(limelight.getTargetArea(), 0.125),
                        0),
                0,
                true);
        // TODO: Find optimal joint angles
        arm.reach(desiredAngles);

    }

}
