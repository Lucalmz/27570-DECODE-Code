package org.firstinspires.ftc.teamcode.pedroPathing.Services; // 你可以根据自己的项目结构修改包名

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * 一个用于测量和显示FTC OpMode主循环频率的工具类.
 *
 * 使用方法:
 * 1. 在你的OpMode中创建这个类的一个实例.
 *    `private LoopFrequencyMonitor loopMonitor = new LoopFrequencyMonitor();`
 * 2. 在你的 `loop()` 方法的结尾处, 调用 `displayTelemetry()` 方法.
 *    `loopMonitor.displayTelemetry(telemetry);`
 */
public class LoopFrequencyMonitor {

    // 用于计时的ElapsedTime对象
    private final ElapsedTime timer;

    // 存储上一次循环的时间点
    private double lastLoopTimeSecs = 0;

    // 计算出的循环频率
    private double loopFrequency = 0;

    // 计算出的单次循环时间
    private double loopTimeMillis = 0;

    /**
     * 构造函数，初始化计时器.
     */
    public LoopFrequencyMonitor() {
        timer = new ElapsedTime();
        timer.reset();
    }

    /**
     * 更新频率计算. 这个方法应该在每次循环的开始或结尾被调用.
     */
    private void update() {
        double currentTimeSecs = timer.seconds();
        double deltaLoopTimeSecs = currentTimeSecs - lastLoopTimeSecs;
        lastLoopTimeSecs = currentTimeSecs;

        // 避免除以零的错误
        if (deltaLoopTimeSecs > 0) {
            loopFrequency = 1.0 / deltaLoopTimeSecs;
            loopTimeMillis = deltaLoopTimeSecs * 1000;
        }
    }

    /**
     * 将循环频率和时间信息添加到遥测数据中.
     * 这个方法会自动调用update()来刷新数据.
     *
     * @param telemetry 你的OpMode中的telemetry对象.
     */
    public void displayTelemetry(Telemetry telemetry) {
        // 首先更新数据
        update();

        // 添加数据到遥测
        telemetry.addData("Loop Frequency (Hz)", String.format("%.2f", loopFrequency));
        telemetry.addData("Loop Time (ms)", String.format("%.2f", loopTimeMillis));
    }

    /**
     * 获取当前的循环频率 (Hz).
     * @return double 循环频率.
     */
    public double getLoopFrequency() {
        return loopFrequency;
    }

    /**
     * 获取当前的单次循环时间 (毫秒).
     * @return double 循环时间.
     */
    public double getLoopTimeMillis() {
        return loopTimeMillis;
    }
}