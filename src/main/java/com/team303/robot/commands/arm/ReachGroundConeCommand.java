package com.team303.robot.commands.arm;

import com.team303.robot.subsystems.ArmSubsystem;
import com.team303.robot.subsystems.PhotonvisionModule;
import com.team303.robot.subsystems.PoseEstimatorModule;
import com.team303.robot.subsystems.SwerveSubsystem;
import com.team303.robot.subsystems.PhotonvisionModule.PhotonPipeline;
import com.team303.robot.subsystems.PhotonvisionModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReachGroundConeCommand extends CommandBase {
    private static final ArmSubsystem arm = ArmSubsystem.getArm();
    private static final SwerveSubsystem swerve = SwerveSubsystem.getSwerve();
    private static final PoseEstimatorModule poseEstimator = PoseEstimatorModule.getPoseSubsystem();
    public static PIDController xControl;
    public static PIDController yControl;

    public ReachGroundConeCommand() {
        addRequirements(swerve,arm);
        xControl = new PIDController(0.01,0,0);
        yControl = new PIDController(0.01,0,0);
    }

    @Override
    public void execute() {
        if (PhotonvisionModule.getPipeline() != PhotonPipeline.CONE) {
            PhotonvisionModule.setPipeline(PhotonPipeline.CONE);
        }
        //TODO: Find optimal distance for drivetrain from cone
        swerve.drive(
            new Translation2d(
            xControl.calculate(PhotonvisionModule.getBestTarget().getBestCameraToTarget().getX(),Units.inchesToMeters(5)),
            yControl.calculate(PhotonvisionModule.getBestTarget().getBestCameraToTarget().getY(),0)
            ),
            0,
            true
        );
        Translation3d armToCone = poseEstimator.getArmtoTargetTranslation(PhotonPipeline.CONE);
        //TODO: Find optimal part of cone to grab
        arm.reach(armToCone.plus(new Translation3d()));
    }
    
}
