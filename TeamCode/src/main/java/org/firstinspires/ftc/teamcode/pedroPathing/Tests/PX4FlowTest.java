package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Models.PX4Flow;

@TeleOp(name = "PX4Flow Test", group = "Sensor Tests")
@Disabled
public class PX4FlowTest extends OpMode {

    private PX4Flow px4Flow;

    @Override
    public void init() {
        // Initialize the sensor
        px4Flow = hardwareMap.get(PX4Flow.class, "px4flow");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        if(!px4Flow.isConnected()) {
            telemetry.addData("Status", "Not Connected");
            telemetry.update();
        }
    }
    public void loop(){
        if(px4Flow.isConnected()) {
            PX4Flow.IntegralFrame frame = px4Flow.getIntegralFrame();
            telemetry.addData("Status", "Connected");
            telemetry.addData("Flow X", frame.pixelFlowXIntegral);
            telemetry.addData("Flow Y", frame.pixelFlowYIntegral);
            telemetry.addData("Gyro X", frame.gyroXRateIntegral);
            telemetry.addData("Gyro Y", frame.gyroYRateIntegral);
            telemetry.addData("Gyro Z", frame.gyroZRateIntegral);
            telemetry.addData("Frame Count", frame.frameCountSinceLastReadout);
            telemetry.addData("Integration Time", frame.integrationTimespanMicroseconds);
            telemetry.addData("Sonar Timestamp", frame.sonarTimestampMicroseconds);
            telemetry.addData("Ground Distance", frame.groundDistance);
            telemetry.addData("Gyro Temperature", frame.gyroTemperatureCelsius);
            telemetry.addData("Quality", frame.quality);
            telemetry.addData("Rotation",px4Flow.getSensorRotation());
            telemetry.update();
        }
    }
}