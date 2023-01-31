package com.team303.lib.kinematics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.MatBuilder;

public class InverseDynamicsInjector {
        private List<List<Double>> anglePositions = new ArrayList<List<Double>>();
        private List<List<Double>> angularVelocities = new ArrayList<List<Double>>();
        private List<List<Double>> angularAccelerations = new ArrayList<List<Double>>();
        private List<Matrix<N3, N3>> inertiaMatrices = new ArrayList<Matrix<N3, N3>>();
        private List<Matrix<N3, N1>> angularAccelerationVectors = new ArrayList<Matrix<N3, N1>>();
        private MatBuilder<N3, N1> angularAccelerationVectorBuilder;
        private List<Matrix<N3, N1>> centrifugalTermsVectors = new ArrayList<Matrix<N3, N1>>();
        private List<Matrix<N3, N1>> gravityTermsVectors = new ArrayList<Matrix<N3, N1>>();
        private List<Matrix<N3, N1>> jointMomentVectors = new ArrayList<Matrix<N3, N1>>();
        private List<Double> segmentMasses = new ArrayList<Double>();
        private List<Double> jointToCenterOfMass = new ArrayList<Double>();
        private List<Double> segmentLengths = new ArrayList<Double>();
        private List<Double> inertiaMoments = new ArrayList<Double>();
        private List<List<Double>> torques = new ArrayList<List<Double>>();
        // Units in in./sec.^2
        private static final double GRAVITY_CONSTANT = 386.09;

        public InverseDynamicsInjector(RMACProfile profile, List<Double> segmentMasses,
                        List<Double> jointToCenterOfMass,
                        List<Double> segmentLengths, List<Double> angleAccel) {
                this.segmentMasses = segmentMasses;
                this.jointToCenterOfMass = jointToCenterOfMass;
                this.segmentLengths = segmentLengths;
                for (int i = 0; i < angularAccelerations.get(0).size(); i++) {
                        inertiaMoments.add(segmentLengths.get(0) * segmentLengths.get(0) * segmentMasses.get(0)
                                        / angularVelocities.get(i).get(0));
                        inertiaMoments.add(segmentLengths.get(1) * segmentLengths.get(1) * segmentMasses.get(1)
                                        / angularVelocities.get(i).get(1));
                        inertiaMoments.add(segmentLengths.get(2) * segmentLengths.get(2) * segmentMasses.get(2)
                                        / angularVelocities.get(i).get(2));
                        Matrix<N3, N3> inertiaMatrix = new Matrix<>(Nat.N3(), Nat.N3());
                        Matrix<N3, N1> centrifugalTermsVector = new Matrix<>(Nat.N3(), Nat.N1());
                        Matrix<N3, N1> gravityTermsVector = new Matrix<>(Nat.N3(), Nat.N1());

                        inertiaMatrices.add(inertiaMatrix);
                        centrifugalTermsVectors.add(centrifugalTermsVector);
                        gravityTermsVectors.add(gravityTermsVector);
                        List<Double> interpolationAnglePositions = profile.getFinalJointPositions().get(i);
                        List<Double> interpolationAngleVelocities = profile.getFinalJointVelocities().get(i);
                        List<Double> interpolationAngleAccelerations = profile.getFinalJointAccelerations().get(i);
                        Matrix<N3, N1> angularAccelerationVectors = angularAccelerationVectorBuilder.fill(
                                        interpolationAngleAccelerations.get(0), interpolationAngleAccelerations.get(1),
                                        interpolationAngleAccelerations.get(2));
                        double Malpha0_1 = segmentMasses.get(1)
                                        * (jointToCenterOfMass.get(1) * jointToCenterOfMass.get(1)
                                                        + segmentLengths.get(0)
                                                        + jointToCenterOfMass.get(1)
                                                        + Math.cos(interpolationAnglePositions.get(1)))
                                        + inertiaMoments.get(1)
                                        + segmentMasses.get(2) * (segmentLengths.get(1) * segmentLengths.get(1)
                                                        + jointToCenterOfMass.get(2) * jointToCenterOfMass.get(2)
                                                        + segmentLengths.get(0) * segmentLengths.get(1)
                                                                        * Math.cos(interpolationAnglePositions.get(1))
                                                        + segmentLengths.get(0) * jointToCenterOfMass.get(2)
                                                                        * Math.cos(interpolationAnglePositions.get(1)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(2))
                                                        + 2 * segmentLengths.get(1) * jointToCenterOfMass.get(2)
                                                                        * Math.cos(interpolationAnglePositions.get(2)))
                                        + inertiaMoments.get(2);
                        double Malpha0_2 = segmentMasses.get(2)
                                        * (jointToCenterOfMass.get(2) * jointToCenterOfMass.get(2)
                                                        + segmentLengths.get(0) * jointToCenterOfMass.get(2)
                                                                        * Math.cos(interpolationAnglePositions.get(1)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(2))
                                                        + segmentLengths.get(1) * jointToCenterOfMass.get(2)
                                                                        * Math.cos(interpolationAnglePositions.get(2)))
                                        + inertiaMoments.get(2);
                        double Malpha1_2 = segmentMasses.get(2)
                                        * (jointToCenterOfMass.get(2) * jointToCenterOfMass.get(2)
                                                        + segmentLengths.get(1) * jointToCenterOfMass.get(2)
                                                                        * Math.cos(interpolationAnglePositions.get(2))
                                                        + inertiaMoments.get(2));
                        inertiaMatrix.set(0, 1, Malpha0_1);
                        inertiaMatrix.set(1, 0, Malpha0_1);
                        inertiaMatrix.set(0, 2, Malpha0_2);
                        inertiaMatrix.set(2, 0, Malpha0_2);
                        inertiaMatrix.set(1, 2, Malpha1_2);
                        inertiaMatrix.set(2, 1, Malpha1_2);
                        inertiaMatrix.set(0, 0, segmentMasses.get(0) * jointToCenterOfMass.get(0)
                                        * jointToCenterOfMass.get(0)
                                        + inertiaMoments.get(0)
                                        + segmentMasses.get(1) * (segmentLengths.get(0) * segmentLengths.get(0)
                                                        + jointToCenterOfMass.get(1) * jointToCenterOfMass.get(1)
                                                        + 2 * segmentLengths.get(0)
                                                                        * jointToCenterOfMass.get(1)
                                                                        * Math.cos(interpolationAnglePositions.get(1))
                                                        + inertiaMoments.get(1)
                                                        + segmentMasses.get(2) * (segmentLengths.get(0)
                                                                        * segmentLengths.get(0)
                                                                        + segmentLengths.get(1) * segmentLengths.get(1)
                                                                        + segmentLengths.get(1)
                                                                        + jointToCenterOfMass.get(2)
                                                                                        * jointToCenterOfMass.get(2)
                                                                        + 2 * segmentLengths.get(0)
                                                                                        * segmentLengths.get(1)
                                                                                        * Math.cos(interpolationAnglePositions
                                                                                                        .get(1))
                                                                        + 2 * segmentLengths.get(0)
                                                                                        * jointToCenterOfMass.get(2)
                                                                                        * Math.cos(interpolationAnglePositions
                                                                                                        .get(1)
                                                                                                        + interpolationAnglePositions
                                                                                                                        .get(2))
                                                                        + 2 * segmentLengths.get(0)
                                                                                        * jointToCenterOfMass.get(2)
                                                                                        * Math.cos(interpolationAnglePositions
                                                                                                        .get(1)
                                                                                                        + interpolationAnglePositions
                                                                                                                        .get(2))
                                                                        + 2 * segmentLengths.get(1)
                                                                                        * jointToCenterOfMass.get(2)
                                                                                        * Math.cos(interpolationAnglePositions
                                                                                                        .get(2)))
                                                        + inertiaMoments.get(2)));
                        inertiaMatrix.set(1, 1, segmentMasses.get(1) * jointToCenterOfMass.get(1)
                                        * jointToCenterOfMass.get(1)
                                        + inertiaMoments
                                                        .get(1)
                                        + segmentMasses.get(2) * (segmentLengths.get(1) * segmentLengths.get(1)
                                                        + jointToCenterOfMass.get(2) * jointToCenterOfMass.get(2)
                                                        + segmentLengths.get(1)
                                                                        * jointToCenterOfMass.get(2)
                                                                        * Math.cos(interpolationAnglePositions.get(2)))
                                        + inertiaMoments.get(2));
                        inertiaMatrix.set(2, 2,
                                        segmentMasses.get(2) * jointToCenterOfMass.get(2) * jointToCenterOfMass.get(2)
                                                        + inertiaMoments.get(2));
                        centrifugalTermsVector.set(0, 0,
                                        -((segmentMasses.get(1) * segmentLengths.get(0) * jointToCenterOfMass.get(1)
                                                        + segmentMasses.get(2) * segmentLengths.get(0)
                                                                        * segmentLengths.get(1))
                                                        * Math.sin(interpolationAnglePositions.get(1))
                                                        + segmentMasses.get(2)
                                                                        * segmentLengths.get(0)
                                                                        * jointToCenterOfMass.get(2)
                                                                        * Math.sin(interpolationAnglePositions.get(1)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(2)))
                                                        * (2 * interpolationAngleVelocities.get(0)
                                                                        * interpolationAngleVelocities.get(1)
                                                                        + interpolationAngleVelocities.get(1)
                                                                                        * interpolationAngleVelocities
                                                                                                        .get(1))
                                                        * -(segmentMasses.get(2) * segmentLengths.get(0)
                                                                        * jointToCenterOfMass.get(2)
                                                                        * Math.sin(interpolationAnglePositions.get(1)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(2))
                                                                        + segmentMasses.get(2) * segmentLengths.get(1)
                                                                                        * jointToCenterOfMass.get(2)
                                                                                        * Math.sin(interpolationAnglePositions
                                                                                                        .get(2)))
                                                        * (2 * interpolationAngleVelocities.get(0)
                                                                        * interpolationAngleVelocities.get(2)
                                                                        + 2 * interpolationAngleVelocities.get(1)
                                                                                        * interpolationAngleVelocities
                                                                                                        .get(2)
                                                                        + interpolationAngleVelocities.get(2)
                                                                                        * interpolationAngleVelocities
                                                                                                        .get(2)));
                        centrifugalTermsVector.set(1, 0,
                                        ((segmentMasses.get(2) * segmentLengths.get(0) * segmentLengths.get(1)
                                                        + segmentMasses.get(1) * jointToCenterOfMass.get(1)
                                                                        * segmentLengths.get(0))
                                                        * Math.sin(interpolationAnglePositions.get(1))
                                                        + segmentMasses.get(2) * jointToCenterOfMass.get(2)
                                                                        * segmentLengths.get(0)
                                                                        * Math.sin(interpolationAnglePositions.get(1)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(2)))
                                                        * interpolationAngleVelocities.get(0)
                                                        * interpolationAngleVelocities.get(0)
                                                        * -segmentMasses.get(2) * jointToCenterOfMass.get(2)
                                                        * segmentLengths.get(1)
                                                        * Math.sin(interpolationAnglePositions.get(2))
                                                        * (2 * interpolationAngleVelocities.get(0)
                                                                        * interpolationAngleVelocities.get(2)
                                                                        + 2 * interpolationAngleVelocities.get(1)
                                                                                        * interpolationAngleVelocities
                                                                                                        .get(2)
                                                                        + interpolationAngleVelocities.get(2)
                                                                                        * interpolationAngleVelocities
                                                                                                        .get(2)));
                        centrifugalTermsVector.set(2, 0,
                                        ((segmentMasses.get(2) * segmentLengths.get(0) * jointToCenterOfMass.get(2)
                                                        * Math.sin(interpolationAnglePositions.get(1)
                                                                        + interpolationAnglePositions.get(2))
                                                        + segmentMasses.get(2) * segmentLengths.get(1)
                                                                        * jointToCenterOfMass
                                                                                        .get(2)
                                                                        * Math.sin(interpolationAnglePositions.get(2)))
                                                        * interpolationAngleVelocities.get(0)
                                                        * interpolationAngleVelocities.get(0)
                                                        + segmentMasses.get(2) * segmentLengths.get(1)
                                                                        * jointToCenterOfMass.get(2)
                                                                        * Math.sin(interpolationAnglePositions.get(2))
                                                                        * (2 * interpolationAngleVelocities.get(0)
                                                                                        * interpolationAngleVelocities
                                                                                                        .get(1)
                                                                                        + interpolationAngleVelocities
                                                                                                        .get(1)
                                                                                                        * interpolationAngleVelocities
                                                                                                                        .get(1))));
                        gravityTermsVector.set(0, 0, segmentMasses.get(0) * GRAVITY_CONSTANT
                                        * jointToCenterOfMass.get(0) * Math.cos(interpolationAnglePositions.get(0))
                                        + segmentMasses.get(1) * GRAVITY_CONSTANT * (segmentLengths.get(0)
                                                        * Math.cos(interpolationAnglePositions.get(0))
                                                        + jointToCenterOfMass.get(1)
                                                                        * Math.cos(interpolationAnglePositions.get(0)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(1)))
                                        + segmentMasses.get(2) * GRAVITY_CONSTANT * (segmentLengths.get(0)
                                                        * Math.cos(interpolationAnglePositions.get(0))
                                                        + segmentLengths.get(1) * Math.cos(interpolationAnglePositions
                                                                        .get(0) + interpolationAnglePositions.get(1))
                                                        + jointToCenterOfMass.get(2)
                                                                        * Math.cos(interpolationAnglePositions.get(0)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(1)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(2))));
                        gravityTermsVector.set(1, 0, segmentMasses.get(1) * GRAVITY_CONSTANT
                                        * jointToCenterOfMass.get(1)
                                        * Math.cos(interpolationAnglePositions.get(0)
                                                        + interpolationAnglePositions.get(1))
                                        + segmentMasses.get(2) * GRAVITY_CONSTANT * (segmentLengths.get(1)
                                                        * Math.cos(interpolationAnglePositions.get(0)
                                                                        + interpolationAnglePositions.get(1))
                                                        + jointToCenterOfMass.get(2)
                                                                        * Math.cos(interpolationAnglePositions.get(0)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(1)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(2))));
                        gravityTermsVector.set(2, 0,
                                        segmentMasses.get(2) * GRAVITY_CONSTANT * jointToCenterOfMass.get(2)
                                                        * Math.cos(interpolationAnglePositions.get(0)
                                                                        + interpolationAnglePositions.get(1)
                                                                        + interpolationAnglePositions.get(2)));
                        Matrix<N3, N1> jointMomentVector = inertiaMatrix.times(angularAccelerationVectors)
                                        .plus(centrifugalTermsVector).plus(gravityTermsVector);
                        jointMomentVectors.add(jointMomentVector);
                        List<Double> interpolationTorques = new ArrayList<>();
                        interpolationTorques.add(jointMomentVector.get(0, 0) - jointMomentVector.get(1, 0)
                                        + (segmentLengths.get(0) - jointToCenterOfMass.get(0))
                                                        * Math.sin(interpolationAnglePositions.get(0))
                                        - (segmentLengths.get(0) - jointToCenterOfMass.get(0))
                                                        * Math.cos(interpolationAnglePositions.get(0))
                                        + jointToCenterOfMass.get(0) * Math.sin(interpolationAnglePositions.get(0))
                                        - jointToCenterOfMass.get(0) * Math.cos(interpolationAnglePositions.get(0)));
                        interpolationTorques.add(jointMomentVector.get(1, 0)
                                        - jointMomentVector.get(2, 0)
                                        + (segmentLengths.get(1) - segmentLengths.get(1)) * Math
                                                        .sin(interpolationAnglePositions.get(0)
                                                                        + interpolationAnglePositions.get(1))
                                        - (segmentLengths.get(1) - jointToCenterOfMass.get(1))
                                                        * Math.cos(interpolationAnglePositions.get(0)
                                                                        + interpolationAnglePositions.get(1))
                                                        + jointToCenterOfMass.get(1)
                                                                        * Math.sin(interpolationAnglePositions.get(0)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(1))
                                                        - jointToCenterOfMass.get(1)
                                                                        * Math.cos(interpolationAnglePositions.get(0)
                                                                                        + interpolationAnglePositions
                                                                                                        .get(1))
                                                        - inertiaMoments.get(1)
                                                                        * interpolationAngleAccelerations.get(0));
                        interpolationTorques.add(jointMomentVector.get(2,0)+jointToCenterOfMass.get(2)*Math.sin(interpolationAnglePositions.get(0)+interpolationAnglePositions.get(1)+interpolationAnglePositions.get(2))-jointToCenterOfMass.get(2)*Math.cos(interpolationAnglePositions.get(0)+interpolationAnglePositions.get(1)+interpolationAnglePositions.get(2))-inertiaMoments.get(2)*interpolationAngleAccelerations.get(0)-inertiaMoments.get(2)*interpolationAngleAccelerations.get(1));
                        torques.add(interpolationTorques);
                }
        }
        public List<List<Double>> getTorques() {
                return torques;
        }

}
