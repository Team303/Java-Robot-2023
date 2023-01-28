package com.team303.lib.kinematics;

import java.util.List;

public interface Interpolator {

    public static float getLength(List<Float> startEffector, List<Float> endEffector) {
        return 0.0f;
    }

    public static float getLength(List<Float> startEffector, Float[] position) {
        return 0.0f;
    }

    public static Float[] interpolate(List<Float> startEffector, List<Float> endEffector, float value) {
        return new Float[] { 0.0f, 0.0f };
    }

}
