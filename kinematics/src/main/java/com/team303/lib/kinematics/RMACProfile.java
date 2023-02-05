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
    private List<Double> maxAccelerationsX = new ArrayList<Double>();
    private List<Double> maxAccelerationsZ = new ArrayList<Double>();
    private double pathLengthInches;
    private double operationTime;
    private double interpolationPointNum;
    private List<Matrix<N4, N4>> transformationMatrices;
    private List<Double> interpolationTimes;
    private List<Double> interpolationCartesianAccelerations = new ArrayList<Double>();
    private List<List<Double>> jointAnglePositions = new ArrayList<List<Double>>();
    private List<List<Double>> jointAngleVelocities = new ArrayList<List<Double>>();
    private List<List<Double>> jointAngleAccelerations = new ArrayList<List<Double>>();
    private List<List<Double>> jointTorques = new ArrayList<List<Double>>();
    private Properties properties;
    private List<Double> clawVoltage;
    private List<Double> elbowVoltage;
    private List<Double> shoulderVoltage;

    public static class Properties {
        public final List<Double> segmentMasses;
        public final List<Double> jointToCenterOfMass;
        public final List<Double> segmentLengthsInches;

        public Properties(List<Double> segmentMasses, List<Double> jointToCenterOfMass,
                List<Double> segmentLengthsInches) {
            this.segmentMasses = segmentMasses;
            this.jointToCenterOfMass = jointToCenterOfMass;
            this.segmentLengthsInches = segmentLengthsInches;
        }
    }

    private boolean accelerationConstraintResolved = false;

    public RMACProfile(FabrikController chain, Properties properties, EffectorPathPlanner path) {
        this.stepSize = (float) path.stepSizeInches;
        this.startEffector = path.startEffector;
        this.endEffector = path.endEffector;
        //TODO: Find a good linear function to represent max acceleration based on path length and neo stall torque.
        this.maxAcceleration = 0;
        this.pathLengthInches = LinearInterpolator.getLength(this.startEffector, this.endEffector);
        this.maxVelocity = Math.sqrt(2 * pathLengthInches * maxAcceleration / Math.PI);
        this.operationTime = Math.sqrt(2 * Math.PI * pathLengthInches / maxAcceleration);
        this.interpolationPointNum = pathLengthInches / stepSize;
        this.properties = properties;
        if (interpolationPointNum != path.getInterpolationPositions().size()) {
            throw new RuntimeException("Number of interpolation points do not match up");
        }
        calculateAccelerations(chain, properties, path);
        accelerationToTorque();
        torqueToVolts();
    }

    public List<Double> getFinalInterpolationAccelerations() {
        return interpolationCartesianAccelerations;
    }

    public List<List<Double>> getFinalJointPositions() {
        return jointAnglePositions;
    }

    public List<List<Double>> getFinalJointVelocities() {
        double deltaT;
        for (int i = 0; i < jointAnglePositions.size() - 1; i++) {
            deltaT = interpolationTimes.get(i + 1) - interpolationTimes.get(i);
            jointAngleVelocities.add(new ArrayList<>());
            for (int j = 0; j < jointAnglePositions.get(i).size(); j++) {
                jointAngleVelocities.get(i).add(jointAnglePositions.get(i).get(j) / deltaT);
            }
        }
        return jointAngleVelocities;
    }

    public List<List<Double>> getFinalJointAccelerations() {
        double deltaT;
        for (int i = 0; i < jointAnglePositions.size() - 1; i++) {
            deltaT = interpolationTimes.get(i + 1) - interpolationTimes.get(i);
            jointAngleAccelerations.add(new ArrayList<>());
            for (int j = 0; j < jointAnglePositions.get(i).size(); j++) {
                jointAngleAccelerations.get(i).add(2 * (jointAnglePositions.get(i).get(j) / deltaT) / deltaT);
            }
        }
        return jointAngleAccelerations;
    }

    public List<List<Double>> getFinalVolts() {
        List<List<Double>> outputVolts = new ArrayList<List<Double>>();
        for (int i = 0; i < clawVoltage.size(); i++) {
            outputVolts.get(i).add(shoulderVoltage.get(i));
            outputVolts.get(i).add(elbowVoltage.get(i));
            outputVolts.get(i).add(clawVoltage.get(i));
        }
        return outputVolts;
    }

    private void calculateAccelerations(FabrikController chain, Properties properties, EffectorPathPlanner path) {
        for (int i = 0; i < path.getInterpolationPositions().size(); i++) {
            Float[] interpolationPosition = path.getInterpolationPositions().get(i);
            interpolationTimes.add(findInterpolationTime(4, interpolationPosition));
            interpolationCartesianAccelerations.add(getInterpolationAcceleration(interpolationTimes.get(i)));
            chain.solveTargetIK(interpolationPosition[0], interpolationPosition[1]);
            List<Double> anglesDegrees = chain.getIKAnglesDegrees();
            jointAnglePositions.add(anglesDegrees);
            TransformationMatrixGenerator armTransformationMatrixInterpolation = new TransformationMatrixGenerator(
                    anglesDegrees, properties.segmentLengthsInches);
            transformationMatrices.add(armTransformationMatrixInterpolation.getAffineTransformationMatrix());
        }
        for (int i = 0; i < path.getInterpolationPositions().size() - 1; i++) {
            maxAccelerationsX.add(maxAcceleration
                    * (transformationMatrices.get(i + 1).get(0, 3) - transformationMatrices.get(i).get(0, 3))
                    / stepSize);
            maxAccelerationsZ.add(maxAcceleration
                    * (transformationMatrices.get(i + 1).get(0, 3) - transformationMatrices.get(i).get(2, 3))
                    / stepSize);
        }
        while (!accelerationConstraintResolved) {
            boolean repeat = false;
            double deltaX;
            double deltaZ;
            double deltaT;
            for (int i = 0; i < interpolationCartesianAccelerations.size() - 1; i++) {
                deltaX = path.getInterpolationPositions().get(i + 1)[0] - path.getInterpolationPositions().get(i)[0];
                deltaT = interpolationTimes.get(i + 1) - interpolationTimes.get(i);
                deltaZ = path.getInterpolationPositions().get(i + 1)[1] - path.getInterpolationPositions().get(i)[1];
                // Assumes uniform acceleration between each interpolated point
                if (2 * (deltaX / deltaT) / deltaT > maxAccelerationsX.get(i)
                        || 2 * (deltaZ / deltaT) / deltaT > maxAccelerationsZ.get(i)) {
                    interpolationTimes.set(i + 1,
                            interpolationTimes.get(i) + Math.max(
                                    Math.sqrt(deltaX / maxAccelerationsX.get(i)),
                                    Math.sqrt(deltaZ / maxAccelerationsZ.get(i))));
                    operationTime += Math.max(
                            Math.sqrt(deltaX / maxAccelerationsX.get(i)),
                            Math.sqrt(deltaZ / maxAccelerationsZ.get(i)));
                    interpolationCartesianAccelerations.set(i + 1, getInterpolationAcceleration(interpolationTimes.get(i + 1)));
                    repeat = true;
                }
            }
            if (repeat != true) {
                accelerationConstraintResolved = true;
            }
        }
    }

    private List<List<Double>> accelerationToTorque() {
        List<Double> gravityTerms = new ArrayList<>();
        // Units in in./sec.^2
        final double GRAVITY_CONSTANT = 386.09;
        for (int i = 0; i < jointAnglePositions.size() - 1; i++) {
            gravityTerms.add(0, properties.segmentMasses.get(0) * GRAVITY_CONSTANT
                    * properties.jointToCenterOfMass.get(0) * Math.cos(jointAnglePositions.get(i).get(0))
                    + properties.segmentMasses.get(1) * GRAVITY_CONSTANT * (properties.segmentLengthsInches.get(0)
                            * Math.cos(jointAnglePositions.get(i).get(0))
                            + properties.jointToCenterOfMass.get(1)
                                    * Math.cos(jointAnglePositions.get(i).get(0)
                                            + jointAnglePositions.get(i).get(1)))
                    + properties.segmentMasses.get(2) * GRAVITY_CONSTANT * (properties.segmentLengthsInches.get(0)
                            * Math.cos(jointAnglePositions.get(i).get(0))
                            + properties.segmentLengthsInches.get(1)
                                    * Math.cos(jointAnglePositions.get(i).get(0) + jointAnglePositions.get(i).get(1))
                            + properties.jointToCenterOfMass.get(2)
                                    * Math.cos(jointAnglePositions.get(i).get(0)
                                            + jointAnglePositions.get(i).get(1)
                                            + jointAnglePositions.get(i).get(2))));
            gravityTerms.add(1, properties.segmentMasses.get(1) * GRAVITY_CONSTANT
                    * properties.jointToCenterOfMass.get(1)
                    * Math.cos(jointAnglePositions.get(i).get(0)
                            + jointAnglePositions.get(i).get(1))
                    + properties.segmentMasses.get(2) * GRAVITY_CONSTANT * (properties.segmentLengthsInches.get(1)
                            * Math.cos(jointAnglePositions.get(i).get(0)
                                    + jointAnglePositions.get(i).get(1))
                            + properties.jointToCenterOfMass.get(2)
                                    * Math.cos(jointAnglePositions.get(i).get(0)
                                            + jointAnglePositions.get(i).get(1)
                                            + jointAnglePositions.get(i).get(2))));
            gravityTerms.add(2,
                    properties.segmentMasses.get(2) * GRAVITY_CONSTANT * properties.jointToCenterOfMass.get(2)
                            * Math.cos(jointAnglePositions.get(i).get(0)
                                    + jointAnglePositions.get(i).get(1)
                                    + jointAnglePositions.get(i).get(2)));
            for (int j = 0; j < jointAnglePositions.get(i).size(); j++) {
                // Torque = mass * distance from joint to center of mass * angular acceleration
                // + (mass * length * angular velocity^2) / 2
                // Torque of middle segment needs to be summed with furthest segment torque, and
                // torque of base segment needs to be summed with middle segment
                jointTorques.get(i).add(2,
                        (properties.segmentMasses.get(2) * properties.jointToCenterOfMass.get(2)
                                * this.getFinalJointAccelerations().get(i).get(2)
                                + (properties.segmentMasses.get(2) * properties.segmentLengthsInches.get(2)
                                        * this.getFinalJointVelocities().get(i).get(2)
                                        * this.getFinalJointVelocities().get(i).get(2)) / 2)
                                + gravityTerms.get(2));
                jointTorques.get(i).add(1,
                        (properties.segmentMasses.get(1) * properties.jointToCenterOfMass.get(1)
                                * this.getFinalJointAccelerations().get(i).get(1)
                                + (properties.segmentMasses.get(1) * properties.segmentLengthsInches.get(1)
                                        * this.getFinalJointVelocities().get(i).get(1)
                                        * this.getFinalJointVelocities().get(i).get(1)) / 2)
                                + jointTorques.get(i).get(2) + gravityTerms.get(1));
                jointTorques.get(i).add(0,
                        (properties.segmentMasses.get(0) * properties.jointToCenterOfMass.get(0)
                                * this.getFinalJointAccelerations().get(i).get(0)
                                + (properties.segmentMasses.get(0) * properties.segmentLengthsInches.get(0)
                                        * this.getFinalJointVelocities().get(i).get(0)
                                        * this.getFinalJointVelocities().get(i).get(0)) / 2)
                                + jointTorques.get(i).get(1) + gravityTerms.get(0));
            }
        }
        return jointTorques;
    }

    private void torqueToVolts() {
        for (int i = 0; i < jointAngleAccelerations.size(); i++) {
            // Ke=493.9
            // I=T*angular velocity*493.9
            // R=0.072
            // V=T*angularVelocity*493.9*0.072
            clawVoltage.add(
                    jointTorques.get(i).get(2) * Math.toRadians(getFinalJointVelocities().get(i).get(2)) * 35.5608);
            elbowVoltage.add(
                    jointTorques.get(i).get(1) * Math.toRadians(getFinalJointVelocities().get(i).get(1)) * 35.5608);
            shoulderVoltage.add(
                    jointTorques.get(i).get(0) * Math.toRadians(getFinalJointVelocities().get(i).get(0)) * 35.5608);

        }
    }

    // Find interpolation times by solving a function using Newton Downhill's method
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

}
