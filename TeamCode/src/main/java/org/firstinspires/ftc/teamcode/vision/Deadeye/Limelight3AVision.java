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

            LLResultTypes.DetectorResult target = DeadeyeAPI.getTargetClosestToAnchor();

            if (target != null) {
                int targetId = target.getClassId(); // 在识别到内容之后获取最近的球的ID，0对应绿色，1对应紫色

                telemetry.addData("Closest to Anchor", "ID: %d", targetId);

                // 关于获取误差的完整实例 -----------------------------------------------------------
                double[] errors = DeadeyeAPI.calculateAlignmentError(target);
                if (errors != null) {
                    double errorX = errors[0];
                    double errorY = errors[1];
                    telemetry.addData("Alignment Error", "X_Error: %.1f cm, Y_Error: %.1f cm", errorX, errorY);
                // 获取误差实例结束 ----------------------------------------------------------------

                } else {
                    telemetry.addData("Alignment Error", "Cannot be calculated");
                }
            } else {
                telemetry.addData("Target", "No valid targets detected");
            }
            telemetry.update();
        }
        DeadeyeAPI.stop(); // 停止处理释放limelight资源
    }
}