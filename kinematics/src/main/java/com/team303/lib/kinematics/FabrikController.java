package com.team303.lib.kinematics;

import java.util.ArrayList;
import java.util.List;

import au.edu.federation.caliko.FabrikBone2D;
import au.edu.federation.caliko.FabrikChain2D;
import au.edu.federation.caliko.FabrikChain2D.BaseboneConstraintType2D;
import au.edu.federation.caliko.FabrikStructure2D;
import au.edu.federation.utils.Vec2f;
import edu.wpi.first.math.geometry.Translation3d;

public class FabrikController {

    FabrikStructure2D structure = new FabrikStructure2D();
    FabrikChain2D chain = new FabrikChain2D();

    List<Float> segmentRatio = new ArrayList<Float>();
    List<Float> segmentLength = new ArrayList<Float>();
    List<Float[]> segmentAngleConstraint = new ArrayList<>();
    List<Vec2f> segmentInitialDirection = new ArrayList<>();

    float armLength;

    public void setArmLength(float lengthInches) {
        armLength = lengthInches;
    }

    public float getArmLength() {
        return armLength;
    }

    public void setSegmentLengthRatio(int segmentIndex, float ratio) {
        segmentRatio.add(segmentIndex, ratio);
    }

    public List<Float> getSegmentLengthRatios() {
        return segmentRatio;
    }

    public float getSegmentLengthRatio(int segmentIndex) {
        return segmentRatio.get(segmentIndex);
    }

    public void setSegmentLengths() {
        int x = 0;
        for (int i = 0; i < segmentRatio.size(); i++) {
            segmentLength.add(x, segmentRatio.get(i) * armLength);
        }
        float lengthSum = 0.0f;
        for (int i = 0; i < segmentLength.size(); i++) {
            lengthSum += segmentLength.get(i);
        }
        if (lengthSum != armLength) {
            throw new RuntimeException("Invalid lengths: Segment lengths do not add up to arm length");
        }
    }

    public List<Float> getSegmentLengths() {
        return segmentLength;
    }

    public float getSegmentLength(int segmentIndex) {
        return segmentLength.get(segmentIndex);
    }

    public void setAngleConstraint(int segmentIndex, float clockwiseConstraint, float counterclockwiseConstraint) {
        Float[] angleConstraints = new Float[] { clockwiseConstraint, counterclockwiseConstraint };
        segmentAngleConstraint.add(segmentIndex, angleConstraints);
    }

    public Float[] getAngleConstraints(int segmentIndex) {
        return segmentAngleConstraint.get(segmentIndex);
    }

    public void setSegmentInitialDirection(int segmentIndex, double angleRadians) {
        Vec2f output = new Vec2f();
        output.x = (float) Math.cos(angleRadians);
        output.y = (float) Math.sin(angleRadians);
        segmentInitialDirection.add(segmentIndex, output);
    }

    // Returns radians
    // Probably inaccurate, will fix eventually
    public double getSegmentInitialDirectionRadians(int segmentIndex) {
        return Math.atan2(segmentInitialDirection.get(segmentIndex).y, segmentInitialDirection.get(segmentIndex).x);
    }

    public void initializeArm() {
        Vec2f baseEndLoc = segmentInitialDirection.get(0).times(segmentLength.get(0));
        FabrikBone2D base = new FabrikBone2D(new Vec2f(), baseEndLoc);
        base.setClockwiseConstraintDegs(segmentAngleConstraint.get(0)[0]);
        base.setAnticlockwiseConstraintDegs(segmentAngleConstraint.get(0)[1]);
        chain.addBone(base);
        chain.setBaseboneConstraintType(BaseboneConstraintType2D.GLOBAL_ABSOLUTE);
        chain.setBaseboneConstraintUV(segmentInitialDirection.get(0));
        for (int i = 0; i < segmentLength.size(); i++) {
            chain.addConsecutiveConstrainedBone(segmentInitialDirection.get(i), segmentLength.get(i),
                    segmentAngleConstraint.get(i)[0], segmentAngleConstraint.get(i)[1]);
        }
        structure.addChain(chain);
    }

    public void setMaxIterationAttempts(int maxIterations) {
        chain.setMaxIterationAttempts(maxIterations);
    }

    public void getMaxIterationAttempts() {
        chain.getMaxIterationAttempts();
    }

    public void setSolveDistanceThreshold(float toleranceInches) {
        chain.setSolveDistanceThreshold(toleranceInches);
    }

    public void getSolveDistanceThreshold() {
        chain.getSolveDistanceThreshold();
    }

    public void solveTargetIK(Translation3d target) {
        chain.solveForTarget((float) target.getX(), (float) target.getZ());
    }

    public void solveTargetIK(float xPosition, float yPosition) {
        chain.solveForTarget(xPosition, yPosition);
    }

    public List<Float> getEffectorPoint() {
        List<Float> effectorCoordinates = new ArrayList<Float>();
        effectorCoordinates.add(0, chain.getEffectorLocation().x);
        effectorCoordinates.add(1, chain.getEffectorLocation().y);
        return effectorCoordinates;
    }

    // Returns radians
    public List<Double> getIKAnglesRadians() {
        Vec2f baseVectorDirection;
        double baseRadianDirection;
        List<Double> outputRadianAngles = new ArrayList<Double>();
        baseVectorDirection = chain.getBone(0).getDirectionUV().minus(segmentInitialDirection.get(0));
        baseRadianDirection = Math.atan2(baseVectorDirection.y, baseVectorDirection.x);
        if (baseRadianDirection < -Math.PI / 2) {
            baseRadianDirection += Math.PI;
        }
        outputRadianAngles.add(-baseRadianDirection);
        for (int i = 1; i < chain.getNumBones(); i++) {
            outputRadianAngles
                    .add(Math.acos(chain.getBone(i - 1).getDirectionUV().dot(chain.getBone(i).getDirectionUV())));
        }
        return outputRadianAngles;
    }

    public List<Double> getIKAnglesDegrees() {
        Vec2f baseVectorDirection;
        double baseRadianDirection;
        List<Double> outputDegreeAngles = new ArrayList<Double>();
        baseVectorDirection = chain.getBone(0).getDirectionUV().minus(segmentInitialDirection.get(0));
        baseRadianDirection = Math.atan2(baseVectorDirection.y, baseVectorDirection.x);
        if (baseRadianDirection < -Math.PI / 2) {
            baseRadianDirection += Math.PI;
        }
        outputDegreeAngles.add(Math.toDegrees(-baseRadianDirection));
        for (int i = 1; i < chain.getNumBones(); i++) {
            outputDegreeAngles.add(Math.toDegrees(
                    Math.acos(chain.getBone(i - 1).getDirectionUV().dot(chain.getBone(i).getDirectionUV()))));
        }
        return outputDegreeAngles;
    }

    // Returns inches
    public float getIKPositionError() {
        Vec2f forwardKinematics = new Vec2f();
        for (int i = 0; i < chain.getNumBones(); i++) {
            Vec2f vectorDirection = chain.getBone(i).getDirectionUV();
            forwardKinematics = forwardKinematics.plus(vectorDirection.times(chain.getBone(i).length()));
        }
        Vec2f inverseKinematics = chain.getEffectorLocation();
        return Vec2f.distanceBetween(forwardKinematics, inverseKinematics);
    }
}
