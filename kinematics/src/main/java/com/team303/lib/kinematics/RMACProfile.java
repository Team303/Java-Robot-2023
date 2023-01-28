package com.team303.lib.kinematics;

public class RMACProfile {
    private double stepSize;
    private double maxVelocity;
    private double maxAcceleration;
    private double pathLengthInches;
    private double operationTime;
    private double interpolationPointNum;

    public RMACProfile(double stepSize, double pathLengthInches) {
        this.stepSize = stepSize;
        this.pathLengthInches = pathLengthInches;
        this.maxVelocity = Math.sqrt(2*pathLengthInches*maxAcceleration/Math.PI);
        this.operationTime = Math.sqrt(2*Math.PI*pathLengthInches/maxAcceleration);
        this.interpolationPointNum = pathLengthInches/stepSize;

    }

}
