package com.team303.robot.subsystems;


import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.team303.robot.Robot;
import com.team303.robot.subsystems.PhotonvisionModule.PhotonPipeline;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team303.robot.subsystems.SwerveSubsystem;
import com.team303.robot.subsystems.PhotonvisionModule;

public class PoseEstimatorModule extends SubsystemBase {
    
    private static final SwerveSubsystem swerve = SwerveSubsystem.getSwerve();
    //private static final PhotonvisionModule photonvision = PhotonvisionModule.getPhotonvision();
    public final AprilTagFieldLayout aprilTagField;
    public static final ShuffleboardTab tab = Shuffleboard.getTab("Pose Estimation");

    private final Field2d field2d = new Field2d();

    //TODO: Find transformation from camera to robot
    private static final Transform3d CAMERA_TO_ROBOT_TRANSFORM = new Transform3d(new Translation3d(), new Rotation3d());
    private static final Transform3d CAMERA_TO_ARM_TRANSFORM = new Transform3d(new Translation3d(), new Rotation3d());

    public PhotonPoseEstimator visionPoseEstimator;
    public static SwerveDrivePoseEstimator poseEstimator = SwerveSubsystem.getPoseEstimator();
    public static PoseEstimatorModule instance = new PoseEstimatorModule();

    public PoseEstimatorModule() {
        AprilTagFieldLayout initialLayout;
        try {
            initialLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            var alliance = DriverStation.getAlliance();
            initialLayout.setOrigin(alliance == Alliance.Blue ?
                OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
        } catch(IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            initialLayout = null;
        }
        aprilTagField = initialLayout;
        visionPoseEstimator = new PhotonPoseEstimator(
            aprilTagField,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            PhotonvisionModule.getCamera(),
            CAMERA_TO_ROBOT_TRANSFORM.inverse()
        );

        tab.add("Pose", toString()).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    }

    public static PoseEstimatorModule getPoseSubsystem() {
		return instance;
	}

    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    //Sets the pose estimation to a new pose
    public void setRobotPose(Pose2d newPose) {
        poseEstimator.resetPosition(
        Rotation2d.fromDegrees(Robot.getNavX().getAngle()),
        swerve.getModulePositions(),
        newPose
        );
    }
    
    @Override
    public String toString() {
        var pose = getRobotPose();
        return String.format("(%.2f, %.2f) %.2f degrees", 
            pose.getX(), 
            pose.getY(),
            pose.getRotation().getDegrees());
    }

    public Translation3d getArmtoTargetTranslation(PhotonPipeline pipeline) {
		Transform3d camToTarget = PhotonvisionModule.getBestTarget().getBestCameraToTarget(); 
        Pose3d camPose = new Pose3d(getRobotPose()).transformBy(CAMERA_TO_ROBOT_TRANSFORM.inverse());
        Pose3d armPose = camPose.transformBy(CAMERA_TO_ARM_TRANSFORM);
        return armPose.transformBy(camToTarget).getTranslation();
	}

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        visionPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return visionPoseEstimator.update();
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
        if (result.isPresent()) {
            EstimatedRobotPose visionPoseEstimate = result.get();
            poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedPose.toPose2d(), visionPoseEstimate.timestampSeconds);
        }
        field2d.setRobotPose(getRobotPose());
    }
}
