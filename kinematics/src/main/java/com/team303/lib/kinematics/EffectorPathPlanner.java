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
        this.endEffector =  endEffector;
        this.stepSizeInches = stepSizeInches;
        Float[] interpolationCoordinates = new Float[]{0.0f,0.0f};
        for (int linePoint=0;linePoint<Math.ceil(this.getLength(startEffector,endEffector)/stepSizeInches)-1;linePoint+=stepSizeInches) {
            interpolationCoordinates = this.interpolate(startEffector,endEffector,(linePoint+1)*stepSizeInches);
            interpolationPositions.add(linePoint,interpolationCoordinates);

        }
    }
    public float getLength(List<Float> startEffector, List<Float> endEffector) {
        return (float)Math.hypot(Math.abs(endEffector.get(1)-startEffector.get(1)),Math.abs(endEffector.get(0)-startEffector.get(0)));
    }
    public Float[] interpolate(List<Float> startEffector, List<Float> endEffector,float value) {
        return new Float[]{(endEffector.get(0)-startEffector.get(0))/this.getLength(startEffector,endEffector)*value+startEffector.get(0)
                          ,(endEffector.get(1)-startEffector.get(1))/this.getLength(startEffector,endEffector)*value+startEffector.get(1)
                          };
    }

    
}
