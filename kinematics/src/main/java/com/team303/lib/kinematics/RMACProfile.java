package com.team303.lib.kinematics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N4;

public class RMACProfile {
    private float stepSize;
    private double maxVelocity;
    private double maxAcceleration;
    private List<Float> startEffector;
    private List<Float> endEffector;
    private List<Double> maxAccelerationsX;
    private List<Double> maxAccelerationy;
    private List<Double> maxAccelerationsZ;
    private double pathLengthInches;
    private double operationTime;
    private double interpolationPointNum;
    private List<Matrix<N4, N4>> transformationMatrices;
    private List<Double> interpolationTimes;
    private List<Double> interpolationAccelerations;
    private List<List<Double>> jointAnglePositions;
    private List<List<Double>> jointAngleVelocities;
    private List<List<Double>> jointAngleAccelerations;
    private boolean accelerationConstraintResolved = false;

    public RMACProfile(FabrikController chain, double[] segmentLengthsInches, EffectorPathPlanner path) {
        this.stepSize = (float) path.stepSizeInches;
        this.startEffector = path.startEffector;
        this.endEffector = path.endEffector;
        this.pathLengthInches = LinearInterpolator.getLength(this.startEffector, this.endEffector);
        this.maxVelocity = Math.sqrt(2 * pathLengthInches * maxAcceleration / Math.PI);
        this.operationTime = Math.sqrt(2 * Math.PI * pathLengthInches / maxAcceleration);
        this.interpolationPointNum = pathLengthInches / stepSize;
        if (interpolationPointNum != path.getInterpolationPositions().size()) {
            throw new RuntimeException("Number of interpolation points do not match up");
        }
        for (int i = 0; i < path.getInterpolationPositions().size(); i++) {
            Float[] interpolationPosition = path.getInterpolationPositions().get(i);
            interpolationTimes.add(findInterpolationTime(4, interpolationPosition));
            interpolationAccelerations.add(getInterpolationAcceleration(interpolationTimes.get(i)));
            chain.solveTargetIK(interpolationPosition[0], interpolationPosition[1]);
            List<Double> anglesDegrees = chain.getIKAnglesDegrees();
            jointAnglePositions.add(anglesDegrees);
            TransformationMatrixGenerator armTransformationMatrixInterpolation = new TransformationMatrixGenerator(
                    anglesDegrees, segmentLengthsInches);
            transformationMatrices.add(armTransformationMatrixInterpolation.getAffineTransformationMatrix());
        }
        for (int i = 0; i < path.getInterpolationPositions().size() - 1; i++) {
            maxAccelerationsX.add(maxAcceleration
                    * (transformationMatrices.get(i + 1).get(0, 3) - transformationMatrices.get(i).get(0, 3))
                    / stepSize);
            /*maxAccelerationy.add(maxAcceleration
                    * (transformationMatrices.get(i + 1).get(0, 3) - transformationMatrices.get(i).get(1, 3))
                    / stepSize);*/
            maxAccelerationsZ.add(maxAcceleration
                    * (transformationMatrices.get(i + 1).get(0, 3) - transformationMatrices.get(i).get(2, 3))
                    / stepSize);
        }
        while (!accelerationConstraintResolved) {
            boolean repeat = false;
            double deltaX;
            double deltaT;
            double deltaZ;
            for (int i = 0; i < interpolationAccelerations.size() - 1; i++) {
                deltaX = path.getInterpolationPositions().get(i + 1)[0] - path.getInterpolationPositions().get(i)[0];
                deltaT = interpolationTimes.get(i + 1) - interpolationTimes.get(i);
                deltaZ = path.getInterpolationPositions().get(i + 1)[1] - path.getInterpolationPositions().get(i)[1];
                //Assumes uniform acceleration between each interpolated point
                if (2*(deltaX / deltaT)/deltaT > maxAccelerationsX.get(i) || 2*(deltaZ / deltaT)/deltaT > maxAccelerationsZ.get(i)) {
                    interpolationTimes.set(i + 1,
                            interpolationTimes.get(i) + Math.max(
                                    Math.sqrt(deltaX / maxAccelerationsX.get(i)),
                                    Math.sqrt(deltaZ / maxAccelerationsZ.get(i))));
                    operationTime += Math.max(
                            Math.sqrt(deltaX / maxAccelerationsX.get(i)),
                            Math.sqrt(deltaZ / maxAccelerationsZ.get(i)));
                    interpolationAccelerations.set(i + 1, getInterpolationAcceleration(interpolationTimes.get(i + 1)));
                    repeat = true;
                }
            }
            if (repeat != true) {
                accelerationConstraintResolved = true;
            }
        }

    }
    //Find interpolation times by solving a function using Newton Downhill's method
    private double findInterpolationTime(double iterations, Float[] interpolationPosition) {
        float startToInterpolationPosition = LinearInterpolator.getLength(startEffector, interpolationPosition);
        double oldGuess = Math.PI;
        double newGuess = 0;
        for (int i = 0; i < iterations; i++) {
            newGuess = oldGuess - (interpolationTimeFunction(oldGuess, startToInterpolationPosition)
                    / interpolationTimeFunctionDerivative(oldGuess, startToInterpolationPosition));
            oldGuess = newGuess;
        }
        return newGuess;
    }

    private double interpolationTimeFunction(double value, float startToInterpolationPosition) {
        return (2 * startToInterpolationPosition / maxVelocity)
                + (operationTime * Math.sin(2 * Math.PI * value / operationTime) / 2 * Math.PI) - value;
    }

    private double interpolationTimeFunctionDerivative(double value, float startToInterpolationPosition) {
        return (2 * startToInterpolationPosition / maxVelocity)
                + (operationTime * Math.cos(2 * Math.PI * value / operationTime)) - 1;
    }

    private double getInterpolationAcceleration(double time) {
        return maxVelocity * (Math.PI * Math.sin(2 * Math.PI * time / operationTime) / operationTime);
    }

    public List<Double> getFinalInterpolationAccelerations() {
        return interpolationAccelerations;
    }
    public List<List<Double>> getFinalJointPositions() {
        return jointAnglePositions;
    }
    public List<List<Double>> getFinalJointVelocities() {
        double deltaT;
        for (int i = 0; i < jointAnglePositions.size() - 1; i++) {
            deltaT = interpolationTimes.get(i + 1) - interpolationTimes.get(i);
            jointAngleVelocities.add(new ArrayList<>());
            for (int j = 0; j<jointAnglePositions.get(i).size(); j++) {
                jointAngleVelocities.get(i).add(jointAnglePositions.get(i).get(j)/deltaT);
            }
        }
    return jointAngleVelocities;
    }
    public List<List<Double>> getFinalJointAccelerations() {
        double deltaT;
        for (int i = 0; i < jointAnglePositions.size() - 1; i++) {
            deltaT = interpolationTimes.get(i + 1) - interpolationTimes.get(i);
            jointAngleAccelerations.add(new ArrayList<>());
            for (int j = 0; j<jointAnglePositions.get(i).size(); j++) {
                jointAngleAccelerations.get(i).add(2*(jointAnglePositions.get(i).get(j)/deltaT)/deltaT);
            }
        }
    return jointAngleAccelerations;
    }
}
