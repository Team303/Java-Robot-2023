package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToPositionCommand extends CommandBase {
    
    private List<Double> desiredAngles;

    public MoveToPositionCommand(double shoulderAngle, double elbowAngle, double clawAngle) {
        addRequirements(arm);
        desiredAngles = new ArrayList<Double>() {
            {
                add(Math.toRadians(shoulderAngle));
                add(Math.toRadians(elbowAngle));
                add(Math.toRadians(clawAngle));
            }
        };
    }

    @Override
    public void execute() {
        arm.reach(desiredAngles);
    }

}
