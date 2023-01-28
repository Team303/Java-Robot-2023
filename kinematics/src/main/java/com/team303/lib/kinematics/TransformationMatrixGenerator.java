package com.team303.lib.kinematics;

import java.util.ArrayList;
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
    private MatBuilder<N4,N4> matBuilder;
    
    // Generates the transformation matrix for a 3-link planar arm using
    // Denavit-Hartenberg parameters
    public TransformationMatrixGenerator(double[] anglesDegrees, double[] segmentLengthsInches) {
        alpha.add(Units.degreesToRadians(-90.0));
        alpha.add(0.0);
        alpha.add(0.0);
        alpha.add(0.0);
        lengthsInches.add(0, 0.0);
        for (int i = 0; i < anglesDegrees.length ; i++) {
            theta.add(Units.degreesToRadians(anglesDegrees[i]));
        }
        theta.add(0.0);
        for (int i = 1; i <= segmentLengthsInches.length ; i++) {
            lengthsInches.add(segmentLengthsInches[i]);
        }
        for (int i = 0; i < alpha.size(); i++) {
            segmentTransformationMatrix[i] = matBuilder.fill(
                Math.cos(theta.get(i)),-Math.sin(theta.get(i))*Math.cos(alpha.get(i)),Math.sin(theta.get(i))*Math.sin(alpha.get(i)),alpha.get(i)*Math.cos(theta.get(i)),
                Math.sin(theta.get(i)),Math.cos(theta.get(i))*Math.cos(alpha.get(i)),-Math.cos(theta.get(i))*Math.sin(alpha.get(i)),alpha.get(i)*Math.sin(theta.get(i)),
                0                     ,Math.sin(lengthsInches.get(i))               ,Math.cos(alpha.get(i))                        ,0,
                0                     ,0                                            ,0                                             ,1);
        }

    }

    public Matrix<N4, N4> getAffineTransformationMatrix() {
        Matrix<N4, N4> temp = segmentTransformationMatrix[0].times(segmentTransformationMatrix[1]);
        Matrix<N4, N4> temp1 = segmentTransformationMatrix[2].times(segmentTransformationMatrix[3]);
        return temp.times(temp1);
    }

}
