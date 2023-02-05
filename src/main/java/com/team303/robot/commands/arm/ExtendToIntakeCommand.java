package com.team303.robot.commands.arm;

import static com.team303.robot.Robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.ArrayList;
import java.util.List;

public class ExtendToIntakeCommand extends CommandBase {
    List<Double> desiredAngles = new ArrayList<Double>() {
        {
            add(0.0);
            add(90.0);
            add(90.0);
        }
    };

    public ExtendToIntakeCommand() {
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // TODO: Find optimal angles
        arm.reach(desiredAngles);
    }
}
