package org.firstinspires.ftc.teamcode.vision.Deadeye;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight3A Vision", group = "Sensor")
public class Limelight3AVision extends LinearOpMode {

    private Deadeye DeadeyeAPI;

    @Override
    public void runOpMode() throws InterruptedException {
        DeadeyeAPI = new Deadeye(hardwareMap);
        DeadeyeAPI.start();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            DeadeyeAPI.update();

            LLResultTypes.DetectorResult target = DeadeyeAPI.getClosestTarget();

            if (target != null) {
                String targetClassName = target.getClassName();

                double[] coords = DeadeyeAPI.calculateCoordinates(target);
                if (coords != null) {
                    telemetry.addData("Target Info", "'%s' at x: %.1f cm, y: %.1f cm",
                            targetClassName, coords[0], coords[1]);
                }

                double[] errors = DeadeyeAPI.calculateAlignmentError(target);

                if (errors != null) {
                    double errorX = errors[0];
                    double errorY = errors[1];

                    telemetry.addData("PID Error", "X_Error: %.1f cm, Y_Error: %.1f cm", errorX, errorY);
                } else {
                    telemetry.addData("PID Error", "Cannot be calculated");
                }

            } else {
                telemetry.addData("Target", "No valid targets detected");
            }

            telemetry.update();
        }

        DeadeyeAPI.stop();
    }
}