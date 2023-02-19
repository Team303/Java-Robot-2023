package com.team303.lib.kinematics;

import java.util.ArrayList;
import java.util.List;

public class EffectorPathPlanner {
    public List<Float> startEffector;
    public List<Float> endEffector;
    private List<Float[]> interpolationPositions = new ArrayList<>();
    public double stepSizeInches;
    private float pathLengthInches;


    /**
     * 
     * @param startEffector 2D Pose of current effector as a pair of floats, first value represents x, second value represents y.
     * @param endEffector 2D Pose of desired effector as a pair of floats, first value represents x, second value represents y.
     * @param stepSizeInches The frequency of interpolations in inches
     */
    public EffectorPathPlanner(List<Float> startEffector, List<Float> endEffector, float stepSizeInches) {
        this.startEffector = startEffector;
        this.endEffector = endEffector;
        this.stepSizeInches = stepSizeInches;
        LinearInterpolator lerp = new LinearInterpolator();
        this.pathLengthInches = lerp.getLength(startEffector, endEffector);
        Float[] interpolationCoordinates = new Float[] { 0.0f, 0.0f };
        if (stepSizeInches > pathLengthInches) {
            throw new RuntimeException("Path length is shorter than step size. Try setting a shorter step size.");
        }
        for (float linePoint = stepSizeInches; linePoint < pathLengthInches; linePoint += stepSizeInches) {
            interpolationCoordinates = lerp.interpolate(startEffector, endEffector,
                    linePoint * stepSizeInches);
            interpolationPositions.add(interpolationCoordinates);
        }
    }

    /**
     * 
     * @return List of arrays of floats. Each array has two values, the first representing x-coordinate and the
     * second representing y-coordinate.
     */
    public List<Float[]> getInterpolationPositions() {
        return interpolationPositions;
    }

    @Override
    public String toString() {
        return "Start Effector: "+startEffector+"\nInterpolated Positions"+interpolationPositions+"\nEnd Effector: "+endEffector;
    }

}
