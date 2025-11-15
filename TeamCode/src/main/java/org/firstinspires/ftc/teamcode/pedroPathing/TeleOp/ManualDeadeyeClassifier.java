package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.Deadeye.Deadeye;

@TeleOp(name = "Manual Deadeye Classifier", group = "Deadeye")
public class ManualDeadeyeClassifier extends LinearOpMode {

    private Deadeye deadeye;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Deadeye System...");
        telemetry.update();

        deadeye = new Deadeye(hardwareMap);
        deadeye.start();

        telemetry.addLine("System Ready. Press Start to begin.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            deadeye.stop();
            return;
        }

        while (opModeIsActive()) {

            deadeye.update();

            if (gamepad1.a) {
                deadeye.resetIntakeCount();
            }

            telemetry.addData("Is Classifying", deadeye.isClassifying());
            telemetry.addData("Classification Plan", deadeye.getClassificationPlan().toString());
            telemetry.addLine();
            telemetry.addLine("Place pixels in front of the camera.");
            telemetry.addLine("Press 'A' on Gamepad 1 to reset state.");
            telemetry.update();
        }

        deadeye.stop();
    }
}