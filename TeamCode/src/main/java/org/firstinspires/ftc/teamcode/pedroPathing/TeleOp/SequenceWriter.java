package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.bear27570.yuan.BotFactory.Gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ConstantLib.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
@TeleOp(name = "Ball Sequence Writer")
public class SequenceWriter extends OpMode {
    private String currentSequence = GPP;
    private GamepadEx gamepad;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        gamepad = GamepadEx.GetGamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");
        telemetry.addLine("---Launch Sequence--");
        telemetry.addData("Current Sequence", currentSequence);
        telemetry.addLine("G-P-P (X) Cross");
        telemetry.addLine("P-G-P (O) circle");
        telemetry.addLine("P-P-G (Δ) triangle");
        telemetry.update();
        gamepad.update();
        if(gamepad.cross.Pressed()){
            currentSequence=GPP;
        }
        if(gamepad.circle.Pressed()){
            currentSequence=PGP;
        }
        if(gamepad.triangle.Pressed()){
            currentSequence=PPG;
        }
        if(gamepad.square.Pressed()){
            try {
                saveSequenceToFile(currentSequence);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    /**
     * 将字符串保存到文件中
     * @param data 要保存的字符串数据
     */
    private void saveSequenceToFile(String data) throws InterruptedException {
        // AppUtil.ROOT_FOLDER 指向设备上的 /sdcard/FIRST 目录
        File file = new File(AppUtil.ROOT_FOLDER, FILENAME);
        try {
            FileWriter writer = new FileWriter(file, false); // false表示覆盖文件内容
            writer.write(data);
            writer.close();
            telemetry.addLine("成功保存到文件!");
        } catch (IOException e) {
            telemetry.addLine("错误: 文件保存失败!");
            telemetry.addData("错误信息", e.getMessage());
        }
        telemetry.update();
        Thread.sleep(10000000);
        stop();
    }
}
