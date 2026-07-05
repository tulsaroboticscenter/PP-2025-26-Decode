package org.firstinspires.ftc.teamcode.Training;

import org.jetbrains.annotations.NotNull;
/*
    demonstration of a java class to contain reusable methods
    Note: this is NOT an opmode but a collection of methods
    related to robot headings
 */

public class RobotLocation {
    double angleRadians;

    public RobotLocation(double angleDegrees) {
        this.angleRadians = Math.toRadians(angleDegrees);
    }

    public double getHeading(){
        double angle = this.angleRadians;

        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI){
            angle += 2 * Math.PI;
            }

        return Math.toDegrees(angle);
    }

    @Override @NotNull
    public String toString(){
        return "Robot location radians (" + angleRadians + ")";
    }

public void turn(double changeAngleDegrees){
            angleRadians += Math.toRadians(changeAngleDegrees);
    }

    public void setAngle(double angleDegrees){
        this.angleRadians = Math.toRadians(angleDegrees);
    }
}
