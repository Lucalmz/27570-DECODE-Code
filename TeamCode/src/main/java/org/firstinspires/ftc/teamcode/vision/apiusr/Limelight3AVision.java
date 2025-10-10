package org.firstinspires.ftc.teamcode.vision.apiusr;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.vision.limelightTFOD;

@TeleOp(name = "Manual Vision OpMode", group = "Sensor")
public class Limelight3AVision extends LinearOpMode {

    private limelightTFOD visionAPI;

    @Override
    public void runOpMode() throws InterruptedException {
        visionAPI = new limelightTFOD(hardwareMap);
        visionAPI.start();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            visionAPI.update();

            LLStatus status = visionAPI.getStatus();
            telemetry.addData("LL Status", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());

            LLResultTypes.DetectorResult closestGreen = visionAPI.getClosestGreen();
            if (closestGreen != null) {
                double[] coords = visionAPI.calculateCoordinates(closestGreen);
                if (coords != null) {
                    telemetry.addData("Closest Green", "Angle(x:%.1f, y:%.1f) | Coords(x:%.1f, y:%.1f cm)",
                            closestGreen.getTargetXDegrees(), closestGreen.getTargetYDegrees(), coords[0], coords[1]);
                }
            } else {
                telemetry.addData("Closest Green", "Not detected");
            }

            LLResultTypes.DetectorResult closestPurple1 = visionAPI.getClosestPurple();
            if (closestPurple1 != null) {
                double[] coords = visionAPI.calculateCoordinates(closestPurple1);
                if (coords != null) {
                    telemetry.addData("1st Closest Purple", "Angle(x:%.1f, y:%.1f) | Coords(x:%.1f, y:%.1f cm)",
                            closestPurple1.getTargetXDegrees(), closestPurple1.getTargetYDegrees(), coords[0], coords[1]);
                }
            } else {
                telemetry.addData("1st Closest Purple", "Not detected");
            }

            LLResultTypes.DetectorResult closestPurple2 = visionAPI.getSecondClosestPurple();
            if (closestPurple2 != null) {
                double[] coords = visionAPI.calculateCoordinates(closestPurple2);
                if (coords != null) {
                    telemetry.addData("2nd Closest Purple", "Angle(x:%.1f, y:%.1f) | Coords(x:%.1f, y:%.1f cm)",
                            closestPurple2.getTargetXDegrees(), closestPurple2.getTargetYDegrees(), coords[0], coords[1]);
                }
            } else {
                telemetry.addData("2nd Closest Purple", "Not detected");
            }

            telemetry.update();
        }

        visionAPI.stop();
    }
}