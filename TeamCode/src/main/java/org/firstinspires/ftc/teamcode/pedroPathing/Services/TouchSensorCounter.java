package org.firstinspires.ftc.teamcode.pedroPathing.Services;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.Inhale;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.IntakeMotor;
import static org.firstinspires.ftc.teamcode.pedroPathing.library.ObjectLib.Manager;

import com.bear27570.yuan.BotFactory.Model.Action;
import com.bear27570.yuan.BotFactory.Model.ConflictPolicy;
import com.bear27570.yuan.BotFactory.Model.Priority;
import com.bear27570.yuan.BotFactory.ThreadManagement.Task;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 这个类封装了检测触摸传感器“按下后释放”事件的逻辑。
 * 当传感器从“被按下”状态变为“未被按下”状态，并且“未被按下”的持续时间超过一个设定的阈值时，
 * 内部计数器会加一。
 */
public class TouchSensorCounter {

    // 硬件和计时工具
    private final TouchSensor touchSensor;
    private final ElapsedTime releaseTimer;

    // 状态变量
    private int count = 0;
    private boolean wasPressed = false;
    private boolean isTimingRelease = false;
    private final double releaseTimeThreshold;

    /**
     * 构造函数
     *
     * @param sensor            要监控的 TouchSensor 对象。
     * @param releaseTimeMillis 传感器释放后需要等待的最小时间（毫秒），超过这个时间才会计数。
     */
    public TouchSensorCounter(TouchSensor sensor, double releaseTimeMillis) {
        this.touchSensor = sensor;
        this.releaseTimeThreshold = releaseTimeMillis;
        this.releaseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * 这个方法应该在你的 OpMode 的主循环中被持续调用。
     * 它会更新传感器的状态并根据设定的逻辑增加计数器。
     */
    public void update() {
        boolean isPressed = touchSensor.isPressed();

        // 状态转换：从“被按下”到“未被按下”
        // 这是开始计时的触发条件
        if (wasPressed && !isPressed) {
            releaseTimer.reset();
            isTimingRelease = true;
        }

        // 如果我们正在对“释放”状态进行计时
        if (isTimingRelease && !isPressed) {
            // 检查释放时间是否已超过阈值
            if (releaseTimer.milliseconds() > releaseTimeThreshold) {
                count++; // 计数器加一
                isTimingRelease = false; // 停止计时，防止在下一次按下前重复计数
                if (count >= 3) {
                    IntakeMotor.VelocityAct(Action.Out);
                }
            }
        }

        // 如果在计时期间传感器又被按下，则取消计时
        if (isPressed) {
            isTimingRelease = false;
        }

        // 更新上一帧的状态，为下一次循环做准备
        wasPressed = isPressed;
    }

    /**
     * 获取当前的计数值。
     *
     * @return 传感器触发的次数。
     */
    public int getCount() {
        return count;
    }

    /**
     * 将计数器重置为 0。
     */
    public void reset() {
        count = 0;
        wasPressed = false;
        isTimingRelease = false;
    }
}