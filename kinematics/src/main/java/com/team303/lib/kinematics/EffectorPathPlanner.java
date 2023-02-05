package com.team303.lib.kinematics;

import java.util.ArrayList;
import java.util.List;

public class EffectorPathPlanner {
    public List<Float> startEffector;
    public List<Float> endEffector;
    private List<Float[]> interpolationPositions = new ArrayList<>();
    public double stepSizeInches;
    private float pathLengthInches;

    // Takes in two pair of floats (first in pair represents x-coordinate, second in
    // pair represents y-coordinate)
    public EffectorPathPlanner(List<Float> startEffector, List<Float> endEffector, float stepSizeInches) {
        this.startEffector = startEffector;
        this.endEffector = endEffector;
        this.stepSizeInches = stepSizeInches;
        this.pathLengthInches = LinearInterpolator.getLength(startEffector, endEffector);
        Float[] interpolationCoordinates = new Float[] { 0.0f, 0.0f };
        if (stepSizeInches > pathLengthInches) {
            throw new RuntimeException("Path length is shorter than step size. Try setting a shorter step size.");
        }
        for (float linePoint = stepSizeInches; linePoint < pathLengthInches; linePoint += stepSizeInches) {
            interpolationCoordinates = LinearInterpolator.interpolate(startEffector, endEffector,
                    linePoint * stepSizeInches);
            interpolationPositions.add(interpolationCoordinates);
        }
    }

    // Returns a list of arrays of floats, each array has two values, the first
    // representing x-coordinate and the
    // second representing y-coordinate
    public List<Float[]> getInterpolationPositions() {
        return interpolationPositions;
    }

    @Override
    public String toString() {
        return "Start Effector: "+startEffector+"\nInterpolated Positions"+interpolationPositions+"\nEnd Effector: "+endEffector;
    }

}
