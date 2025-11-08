package org.firstinspires.ftc.teamcode.pedroPathing.Services;

public class Calculator {
    public static double DegreeToPitchServo(double degree){
        if(degree<54.9||degree>90){
            throw new IllegalArgumentException("Degree"+degree+"is not available");
        }
        return (90-degree)*0.02857142857142857142857142857143;
    }
}
