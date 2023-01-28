package com.team303.lib.kinematics;

import java.util.List;

public class LinearInterpolator implements Interpolator {
    public static float getLength(List<Float> startEffector, List<Float> endEffector) {
        return (float) Math.hypot(Math.abs(endEffector.get(1) - startEffector.get(1)),
                Math.abs(endEffector.get(0) - startEffector.get(0)));
    }

    public static float getLength(List<Float> startEffector, Float[] position) {
        return (float) Math.hypot(Math.abs(position[1] - startEffector.get(1)),
                Math.abs(position[0] - startEffector.get(0)));
    }

    public static Float[] interpolate(List<Float> startEffector, List<Float> endEffector, float value) {
        return new Float[] {
                (endEffector.get(0) - startEffector.get(0)) / getLength(startEffector, endEffector) * value
                        + startEffector.get(0),
                (endEffector.get(1) - startEffector.get(1)) / getLength(startEffector, endEffector) * value
                        + startEffector.get(1)
        };
    }

}
