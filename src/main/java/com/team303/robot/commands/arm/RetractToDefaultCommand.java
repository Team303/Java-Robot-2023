package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractToDefaultCommand extends CommandBase {
    List<Double> desiredAngles = new ArrayList<Double>() {
        {
            add(Math.toRadians(0));
            add(Math.toRadians(90));
            add(Math.toRadians(90));
        }
    };
    public RetractToDefaultCommand() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // TODO: Find optimal angles
        arm.reach(desiredAngles);
    }
}
