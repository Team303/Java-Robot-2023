package com.team303.lib.kinematics;

import java.util.ArrayList;
import java.util.List;

public class EffectorPathPlanner {
    private List<Float> startEffector;
    private List<Float> endEffector;
    private List<Float[]> interpolationPositions = new ArrayList<>();
    private float stepSizeInches;

    public EffectorPathPlanner(List<Float> startEffector, List<Float> endEffector, float stepSizeInches) {
        this.startEffector = startEffector;
        this.endEffector = endEffector;
        this.stepSizeInches = stepSizeInches;
        Float[] interpolationCoordinates = new Float[] { 0.0f, 0.0f };
        for (int linePoint = 0; linePoint < Math
                .ceil(LinearInterpolator.getLength(startEffector, endEffector) / stepSizeInches)
                - 1; linePoint += stepSizeInches) {
            interpolationCoordinates = LinearInterpolator.interpolate(startEffector, endEffector,
                    (linePoint + 1) * stepSizeInches);
            interpolationPositions.add(linePoint, interpolationCoordinates);

        }
    }

}
