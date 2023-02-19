package com.team303.lib.kinematics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N4;

public class RMACProfile {
    private float stepSize;
    private double maxVelocity;
    private double maxAcceleration;
    private List<Float> startEffector; //coordinates in inches
    private List<Float> endEffector; //coordinates in inches
    private List<Double> maxAccelerationsX = new ArrayList<Double>();
    private List<Double> maxAccelerationsZ = new ArrayList<Double>();
    private double pathLengthInches;
    private double operationTime; //sec.
    private int interpolationPointNum;
    private List<Matrix<N4, N4>> transformationMatrices;
    private List<Double> interpolationTimes; //sec.
    private List<Double> interpolationCartesianAccelerations = new ArrayList<Double>();
    private List<List<Double>> jointAnglePositions = new ArrayList<List<Double>>(); //deg.
    private List<List<Double>> jointAngleVelocities = new ArrayList<List<Double>>(); //deg./sec.
    private List<List<Double>> jointAngleAccelerations = new ArrayList<List<Double>>(); //deg./sec^2.
    private List<List<Double>> jointTorques = new ArrayList<List<Double>>();
    private Properties properties;
    private List<Double> clawVoltage;
    private List<Double> elbowVoltage;
    private List<Double> shoulderVoltage;
    private LinearInterpolator lerp = new LinearInterpolator();

    public static class Properties {
        public final List<Double> segmentMasses; //lbs.
        public final List<Double> jointToCenterOfMass; //in.
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
        /* TODO: Find a good linear function to represent max acceleration based on path
         * length and neo stall torque. 
         */
        this.maxAcceleration = 0;
        this.pathLengthInches = lerp.getLength(this.startEffector, this.endEffector);
        this.maxVelocity = Math.sqrt(2 * pathLengthInches * maxAcceleration / Math.PI);
        this.operationTime = Math.sqrt(2 * Math.PI * pathLengthInches / maxAcceleration);
        this.interpolationPointNum = (int)Math.ceil(pathLengthInches / stepSize);
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

    public void calculateFinalJointAccelerations() {
        double deltaT;
        for (int i = 0; i < jointAnglePositions.size() - 1; i++) {
            deltaT = interpolationTimes.get(i + 1) - interpolationTimes.get(i);
            jointAngleAccelerations.add(new ArrayList<>());
            for (int j = 0; j < jointAnglePositions.get(i).size(); j++) {
                jointAngleAccelerations.get(i).add(2 * (jointAnglePositions.get(i).get(j) / deltaT) / deltaT);
            }
        }
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

    /*
     * Uses math as described by
     * https://journals.sagepub.com/doi/10.1177/1729881417707147#disp-formula10-
     * 1729881417707147
     */
    private void calculateAccelerations(FabrikController chain, Properties properties, EffectorPathPlanner path) {
        /*
         * Generate initial guesses of times at which each interpolation position is
         * achieved
         * Calculate task-space acceleration
         * Solve inverse kinematics of chain to find joint angle positions and generate
         * homogeneous transformation matrix.
         */
        for (int i = 0; i < path.getInterpolationPositions().size(); i++) {
            Float[] interpolationPosition = path.getInterpolationPositions().get(i);
            interpolationTimes.add(findInterpolationTime(4, interpolationPosition));
            interpolationCartesianAccelerations.add(getInterpolationAcceleration(interpolationTimes.get(i)));
            chain.solveTargetIK(interpolationPosition[0], interpolationPosition[1]);
            List<Double> anglesDegrees = chain.getIKAnglesDegrees();
            jointAnglePositions.add(anglesDegrees);
            TransformationMatrixGenerator armTransformationMatrixInterpolation = new TransformationMatrixGenerator(
                    anglesDegrees, properties.segmentLengthsInches);
            transformationMatrices.add(armTransformationMatrixInterpolation.getHomogeneousTransformationMatrix());
        }
        //Set max accelerations in x-direction and z-direction
        for (int i = 0; i < path.getInterpolationPositions().size() - 1; i++) {
            maxAccelerationsX.add(maxAcceleration
                    * (transformationMatrices.get(i + 1).get(0, 3) - transformationMatrices.get(i).get(0, 3))
                    / stepSize);
            maxAccelerationsZ.add(maxAcceleration
                    * (transformationMatrices.get(i + 1).get(0, 3) - transformationMatrices.get(i).get(2, 3))
                    / stepSize);
        }
        /*
         * If the acceleration in a direction is too high, adjust the time at which the interpolation is achieved
         * until the constraint is met. Then, adjust the acceleration and total operation time accordingly.
         */
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
                    interpolationCartesianAccelerations.set(i + 1,
                            getInterpolationAcceleration(interpolationTimes.get(i + 1)));
                    repeat = true;
                }
            }
            if (repeat != true) {
                accelerationConstraintResolved = true;
            }
        }
    }

    private void accelerationToTorque() {
        List<Double> gravityTerms = new ArrayList<>();
        // Units in in./sec.^2
        final double GRAVITY_CONSTANT = 386.09;
        for (int i = 0; i < jointAnglePositions.size() - 1; i++) {
            calculateFinalJointAccelerations();
            //Gravity dynamics equations found at https://www.yumpu.com/en/document/read/33686001/inverse-dynamics-for-a-three-link-planar-chain
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
                /* Torque = mass * distance from joint to center of mass * angular acceleration
                * + (mass * length * angular velocity^2) / 2
                * Source of equation: I made it up (aka it's probably wrong)
                * Torque of elbow is summed with claw torque, and
                * torque of shoulder is summed with elbow
                */
                jointTorques.get(i).add(2,
                        (properties.segmentMasses.get(2) * properties.jointToCenterOfMass.get(2)
                                * jointAngleAccelerations.get(i).get(2)
                                + (properties.segmentMasses.get(2) * properties.segmentLengthsInches.get(2)
                                        * this.getFinalJointVelocities().get(i).get(2)
                                        * this.getFinalJointVelocities().get(i).get(2)) / 2)
                                + gravityTerms.get(2));
                jointTorques.get(i).add(1,
                        (properties.segmentMasses.get(1) * properties.jointToCenterOfMass.get(1)
                                * jointAngleAccelerations.get(i).get(1)
                                + (properties.segmentMasses.get(1) * properties.segmentLengthsInches.get(1)
                                        * this.getFinalJointVelocities().get(i).get(1)
                                        * this.getFinalJointVelocities().get(i).get(1)) / 2)
                                + jointTorques.get(i).get(2) + gravityTerms.get(1));
                jointTorques.get(i).add(0,
                        (properties.segmentMasses.get(0) * properties.jointToCenterOfMass.get(0)
                                * jointAngleAccelerations.get(i).get(0)
                                + (properties.segmentMasses.get(0) * properties.segmentLengthsInches.get(0)
                                        * this.getFinalJointVelocities().get(i).get(0)
                                        * this.getFinalJointVelocities().get(i).get(0)) / 2)
                                + jointTorques.get(i).get(1) + gravityTerms.get(0));
            }
        }
    }

    private void torqueToVolts() {
        for (int i = 0; i < jointAngleAccelerations.size(); i++) {
            /*
             * The Neo velocity constant Kv = 473 (measured in revs/V*min)
             * The back emf constant (Ke) and torque constant (Kt) are equal
             * Ke = Kt = 60 / (2π * Kv) (measured in V*s/radian and N*m/A respectively)
             * 
             * To find the voltage we have to factor back emf into Ohm's law:
             * 
             * V = I * Rw + Eg
             * 
             * where I is the current (A), Rw is the winding resistance (Ω), and Eg is the
             * back emf voltage (V)
             *
             *
             * I is the current (A) being drawn by the motor which is calculated from the
             * torque and torque
             * constants:
             * 
             * I = T / Kt
             * 
             * where T is torque (N*m) and Kt is the motor torque constant (N*m/A).
             * Dividing these cancels out the N*m and leaves us with amps.
             *
             *
             * Rw is the motor's winding frequency (Ω) per motor phase:
             * 
             * Rw = 36.5 mΩ = 0.0365 Ω
             *
             *
             * Eg is the back emf voltage which is calculated from the angular velocity and
             * the motor's
             * back emf constant:
             * 
             * Eg = Ke * ω
             * 
             * where Ke is the back emf constant (V*s/radian), and ω is the angular velocity
             * (radians/s).
             * Multiplying these cancels out the radians and seconds leaving us with volts.
             * 
             * Together we get our final equation:
             * V = (T / Kt) * Rw + Ke * ω
             */

            final double Kv = 473;
            final double Ke = 60 / (2 * Math.PI * Kv);
            final double Kt = Ke;
            final double Rw = 0.0365;

            var torques = jointTorques.get(i);
            var angularVelocities = getFinalJointVelocities().get(i);

            shoulderVoltage.add((torques.get(0) / Kt) * Rw + Ke * Math.toRadians(angularVelocities.get(0)));
            elbowVoltage.add((torques.get(1) / Kt) * Rw + Ke * Math.toRadians(angularVelocities.get(1)));
            clawVoltage.add((torques.get(2) / Kt) * Rw + Ke * Math.toRadians(angularVelocities.get(2)));

        }
    }

    /**
     * Finds interpolation times by finding zeroes of interpolationTimeFunction() using Newton Downhill's method
     * @param iterations How many times a guess is made of zero of expression.
     * @param interpolationPosition 2D coordinates of end effector pose
     * @return Time that each interpolation position is achieved in seconds
     */
    private double findInterpolationTime(double iterations, Float[] interpolationPosition) {
        float startToInterpolationPosition = lerp.getLength(startEffector, interpolationPosition);
        //Initial guess is π
        double oldGuess = Math.PI;
        double newGuess = 0;
        for (int i = 0; i < iterations; i++) {
            newGuess = oldGuess - (interpolationTimeFunction(oldGuess, startToInterpolationPosition)
                    / interpolationTimeFunctionDerivative(oldGuess, startToInterpolationPosition));
            oldGuess = newGuess;
        }
        return newGuess;
    }
    /**
     * 
     * @param value A value of time in seconds.
     * @param startToInterpolationPosition Length between start effector pose and pose of interpolation in inches.
     * @return The result of evaluating equation 16 of 
     * Gao M, Li Z, He Z, Li X. An adaptive velocity planning method for multi-DOF robot manipulators. 
     * International Journal of Advanced Robotic Systems. 2017;14(3). doi:10.1177/1729881417707147
     */
    private double interpolationTimeFunction(double value, float startToInterpolationPosition) {
        return (2 * startToInterpolationPosition / maxVelocity)
                + (operationTime * Math.sin(2 * Math.PI * value / operationTime) / 2 * Math.PI) - value;
    }
    /**
     * Original time function is f(tᵢ)=2Sᵢ/Vᵐᵃˣ+Tsin(2πtᵢ/T)/2π−tᵢ.
     * Take partial derivative w.r.t tᵢ
     * ∂f/∂tᵢ=2Sᵢ/Vᵐᵃˣ+Tcos(2πtᵢ/T)−1 
     * @param value Value of time in seconds.
     * @param startToInterpolationPosition Length between start effector pose and pose of interpolation in inches.
     * @return The result of evaluating derivative of equation 16 of Gao, Li, He et al.
     */
    private double interpolationTimeFunctionDerivative(double value, float startToInterpolationPosition) {
        return (2 * startToInterpolationPosition / maxVelocity)
                + (operationTime * Math.cos(2 * Math.PI * value / operationTime)) - 1;
    }
    /**
     * Equation 9 of Gao,Li, He et al.
     * @param time Time in seconds to calculate acceleration for
     * @return Task-space acceleration of interpolation in in./sec^2
     */
    private double getInterpolationAcceleration(double time) {
        return maxVelocity * (Math.PI * Math.sin(2 * Math.PI * time / operationTime) / operationTime);
    }

}
