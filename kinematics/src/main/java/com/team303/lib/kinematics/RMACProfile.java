package com.team303.lib.kinematics;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N4;

public class RMACProfile {
    private float stepSize;
    private double maxVelocity;
    private double maxAcceleration;
    private List<Float> startEffector;
    private List<Float> endEffector;
    private List<Double> maxAccelerationx;
    private List<Double> maxAccelerationy;
    private List<Double> maxAccelerationz;
    private double pathLengthInches;
    private double operationTime;
    private double interpolationPointNum;
    private List<Matrix<N4, N4>> transformationMatrices;
    private List<Double> interpolationTimes;
    private List<Double> interpolationAccelerations;
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
            double[] anglesDegrees = chain.getIKAnglesDegrees();
            TransformationMatrixGenerator armTransformationMatrixInterpolation = new TransformationMatrixGenerator(
                    anglesDegrees, segmentLengthsInches);
            transformationMatrices.add(armTransformationMatrixInterpolation.getAffineTransformationMatrix());
        }
        for (int i = 0; i < path.getInterpolationPositions().size() - 1; i++) {
            maxAccelerationx.add(maxAcceleration
                    * (transformationMatrices.get(i + 1).get(0, 3) - transformationMatrices.get(i).get(0, 3))
                    / stepSize);
            maxAccelerationy.add(maxAcceleration
                    * (transformationMatrices.get(i + 1).get(0, 3) - transformationMatrices.get(i).get(1, 3))
                    / stepSize);
            maxAccelerationz.add(maxAcceleration
                    * (transformationMatrices.get(i + 1).get(0, 3) - transformationMatrices.get(i).get(2, 3))
                    / stepSize);
        }
        while (!accelerationConstraintResolved) {
            boolean repeat = false;
            for (int i = 0; i < interpolationAccelerations.size() - 1; i++) {
                if ((path.getInterpolationPositions().get(i + 1)[0] - path.getInterpolationPositions().get(i)[0])
                        / Math.pow((interpolationTimes.get(i + 1) - interpolationTimes.get(i)), 2) > maxAccelerationx
                                .get(i)) {
                    interpolationTimes.set(i + 1,
                            interpolationTimes.get(i) + Math.max(
                                    Math.sqrt((path.getInterpolationPositions().get(i + 1)[0]
                                            - path.getInterpolationPositions().get(i)[0]) / maxAccelerationx.get(i)),
                                    Math.sqrt((path.getInterpolationPositions().get(i + 1)[1]
                                            - path.getInterpolationPositions().get(i)[1]) / maxAccelerationz.get(i))));
                    operationTime += Math.max(
                            Math.sqrt((path.getInterpolationPositions().get(i + 1)[0]
                                    - path.getInterpolationPositions().get(i)[0]) / maxAccelerationx.get(i)),
                            Math.sqrt((path.getInterpolationPositions().get(i + 1)[1]
                                    - path.getInterpolationPositions().get(i)[1]) / maxAccelerationz.get(i)));
                    interpolationAccelerations.set(i + 1, getInterpolationAcceleration(interpolationTimes.get(i + 1)));
                    repeat = true;
                }
            }
            if (repeat != true) {
                accelerationConstraintResolved = true;
            }
        }

    }

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
}
