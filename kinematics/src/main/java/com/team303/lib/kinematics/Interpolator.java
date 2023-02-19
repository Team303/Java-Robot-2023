package com.team303.lib.kinematics;

import java.util.List;

public interface Interpolator {

    public float getLength(List<Float> startEffector, List<Float> endEffector);

    public float getLength(List<Float> startEffector, Float[] position);

    public Float[] interpolate(List<Float> startEffector, List<Float> endEffector, float value);

}
