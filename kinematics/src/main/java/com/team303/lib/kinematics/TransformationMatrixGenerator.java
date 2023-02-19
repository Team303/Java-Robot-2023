package com.team303.lib.kinematics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;

public class TransformationMatrixGenerator {
    private List<Double> theta;
    private List<Double> lengthsInches;
    private List<Double> alpha = new ArrayList<>();
    private Matrix<N4, N4>[] segmentTransformationMatrix;
    private MatBuilder<N4, N4> matBuilder;
    
    /**
     * Generates a homogeneous transformation matrix for a 3-link planar chain using Denavit-Hartenberg parameters.
     * Derivation at Equation 1 of 
     * Gao M, Li Z, He Z, Li X. An adaptive velocity planning method for multi-DOF robot manipulators. 
     * International Journal of Advanced Robotic Systems. 2017;14(3). doi:10.1177/1729881417707147
     */
    public TransformationMatrixGenerator(List<Double> anglesDegrees, List<Double> segmentLengthsInches) {
        alpha = Arrays.asList(Math.toRadians(-90.0),0.0,0.0,0.0);
        lengthsInches.add(0, 0.0);
        for (int i = 0; i < anglesDegrees.size() ; i++) {
            theta.add(Units.degreesToRadians(anglesDegrees.get(i)));
        }
        theta.add(0.0);
        for (int i = 1; i <= segmentLengthsInches.size() ; i++) {
            lengthsInches.add(segmentLengthsInches.get(i));
        }
        for (int i = 0; i < alpha.size(); i++) {
            segmentTransformationMatrix[i] = matBuilder.fill(
                Math.cos(theta.get(i)),-Math.sin(theta.get(i))*Math.cos(alpha.get(i)),Math.sin(theta.get(i))*Math.sin(alpha.get(i)),alpha.get(i)*Math.cos(theta.get(i)),
                Math.sin(theta.get(i)),Math.cos(theta.get(i))*Math.cos(alpha.get(i)),-Math.cos(theta.get(i))*Math.sin(alpha.get(i)),alpha.get(i)*Math.sin(theta.get(i)),
                0                     ,Math.sin(lengthsInches.get(i))               ,Math.cos(alpha.get(i))                        ,0,
                0                     ,0                                            ,0                                             ,1);
        }

    }

    /**
     * 
     * @return A 4x4 homogeneous transformation matrix representing a transformation from the base joint to the
     * pose of the end-effector.
     */
    public Matrix<N4, N4> getHomogeneousTransformationMatrix() {
        Matrix<N4, N4> temp = segmentTransformationMatrix[0].times(segmentTransformationMatrix[1]);
        Matrix<N4, N4> temp1 = segmentTransformationMatrix[2].times(segmentTransformationMatrix[3]);
        return temp.times(temp1);
        
    }

}
