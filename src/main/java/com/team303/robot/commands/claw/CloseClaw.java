package com.team303.robot.commands.claw;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team303.robot.Robot;
import static com.team303.robot.Robot.claw;
public class CloseClaw extends CommandBase
{
    public CloseClaw()
    {
    addRequirements(claw);
    } 

    @Override
    public void execute()
    {
       //claw.claw(1.0);
    }
    @Override
    public boolean isFinished()
    {
        return claw.innerLimitReached(); 
    }
    @Override
    public void end(boolean interrupted)
    {
        //claw.claw(0);
    }


}
