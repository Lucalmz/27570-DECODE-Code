/*
    SPDX-License-Identifier: BSD-3-Clause

    Copyright (c) 2024.

    Based on the PX4 Autopilot Project C++ driver:
    https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/optical_flow/px4flow/px4flow.cpp

    Copyright (c) 2013-2019, 2021 PX4 Development Team. All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name PX4 nor the names of its contributors may be used to
       endorse or promote products derived from this software without specific
       prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN

    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.pedroPathing.Models;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * {@link PX4Flow} is the Java driver for the PX4Flow Optical Flow and Sonar sensor.
 * This driver is a port of the official PX4 Autopilot C++ driver.
 *
 * The sensor provides integrated optical flow, gyroscope, and sonar distance data over I2C.
 *
 * @see <a href="https://docs.px4.io/main/en/sensor/px4flow.html">PX4Flow User Guide</a>
 * @see <a href="https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/optical_flow/px4flow/px4flow.cpp">Original C++ Driver</a>
 */
@I2cDeviceType
@DeviceProperties(
        name = "PX4Flow Optical Flow Sensor",
        xmlTag = "PX4Flow",
        description = "PX4Flow Optical Flow and Sonar Sensor"
)
public class PX4Flow extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    // Default 7-bit I2C address for the PX4Flow
    public static final I2cAddr DEFAULT_ADDRESS = I2cAddr.create7bit(0x42);

    // Register address to request an integral frame measurement
    protected static final byte REG_READ_INTEGRAL_FRAME = 0x16;

    // The integral frame consists of 26 bytes
    protected static final int INTEGRAL_FRAME_LENGTH = 26;

    // Conversion factors derived from the C++ driver
    protected static final double INTEGRAL_VALUE_TO_RADIANS = 10000.0;
    protected static final double RAW_DISTANCE_TO_METERS = 1000.0;
    protected static final double RAW_TEMP_TO_CELSIUS = 100.0;

    protected DistanceUnit distanceUnit;
    protected AngleUnit angleUnit;
    protected Rotation rotation;

    /**
     * Holds a complete data frame from the PX4Flow sensor after parsing, conversion, and rotation.
     */
    public static class IntegralFrame {
        /** Integrated flow in radians around the X-axis since the last update. */
        public float pixelFlowXIntegral;
        /** Integrated flow in radians around the Y-axis since the last update. */
        public float pixelFlowYIntegral;
        /** Integrated angular rate in the configured AngleUnit around the X-axis. */
        public float gyroXRateIntegral;
        /** Integrated angular rate in the configured AngleUnit around the Y-axis. */
        public float gyroYRateIntegral;
        /** Integrated angular rate in the configured AngleUnit around the Z-axis. */
        public float gyroZRateIntegral;
        /** The number of camera frames captured since the last readout. */
        public int frameCountSinceLastReadout;
        /** The time period over which the flow and gyro data were integrated, in microseconds. */
        public long integrationTimespanMicroseconds;
        /** The time since the last valid sonar update, in microseconds. */
        public long sonarTimestampMicroseconds;
        /** Ground distance from the sonar in the configured DistanceUnit. */
        public float groundDistance;
        /** The temperature of the gyroscope in degrees Celsius. */
        public float gyroTemperatureCelsius;
        /**
         * A quality metric for the optical flow estimate.
         * A value of 0 indicates poor quality, while 255 is the maximum quality.
         */
        public int quality;
    }

    /**
     * Defines the physical orientation of the sensor relative to the robot's frame.
     * This is used to rotate sensor readings into the robot's coordinate system.
     */
    public enum Rotation {
        ROTATION_NONE,
        ROTATION_YAW_45,
        ROTATION_YAW_90,
        ROTATION_YAW_135,
        ROTATION_YAW_180,
        ROTATION_YAW_225,
        ROTATION_YAW_270,
        ROTATION_YAW_315,
        ROTATION_ROLL_180,
        ROTATION_ROLL_180_YAW_45,
        ROTATION_ROLL_180_YAW_90,
        ROTATION_ROLL_180_YAW_135,
        ROTATION_PITCH_180,
        ROTATION_ROLL_180_YAW_225,
        ROTATION_ROLL_180_YAW_270,
        ROTATION_ROLL_180_YAW_315,
        ROTATION_ROLL_90,
        ROTATION_ROLL_90_YAW_45,
        ROTATION_ROLL_90_YAW_90,
        ROTATION_ROLL_90_YAW_135,
        ROTATION_ROLL_270,
        ROTATION_ROLL_270_YAW_45,
        ROTATION_ROLL_270_YAW_90,
        ROTATION_ROLL_270_YAW_135,
        ROTATION_PITCH_90,
        ROTATION_PITCH_270,
        ROTATION_PITCH_180_YAW_90,
        ROTATION_PITCH_180_YAW_270,
        ROTATION_ROLL_90_PITCH_90,
        ROTATION_ROLL_180_PITCH_90,
        ROTATION_ROLL_270_PITCH_90,
        ROTATION_ROLL_90_PITCH_180,
        ROTATION_ROLL_270_PITCH_180,
        ROTATION_ROLL_90_PITCH_270,
        ROTATION_ROLL_180_PITCH_270,
        ROTATION_ROLL_270_PITCH_270,
        ROTATION_ROLL_90_PITCH_180_YAW_90,
        ROTATION_ROLL_90_YAW_270,
        ROTATION_YAW_293_PITCH_68_ROLL_90,
        ROTATION_PITCH_315,
        ROTATION_ROLL_45,
        ROTATION_ROLL_315,
        ROTATION_DOWNWARD_FACING // Typical orientation for ground tracking
    }

    public PX4Flow(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        deviceClient.setI2cAddress(DEFAULT_ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        this.distanceUnit = DistanceUnit.CM;
        this.angleUnit = AngleUnit.RADIANS;
        this.rotation = Rotation.ROTATION_NONE;
        return isConnected();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "PX4Flow Optical Flow Sensor";
    }

    /**
     * Checks if the sensor is connected by attempting to read data from it.
     * @return True if a valid data frame can be read, false otherwise.
     */
    public boolean isConnected() {
        try {
            // A successful read indicates the device is present and responding.
            readRawIntegralFrame();
            return true;
        } catch (Exception e) {
            // An exception (e.g., timeout) suggests the device is not connected.
            return false;
        }
    }

    /**
     * Sets the distance unit for ground distance measurements.
     * @param unit The desired distance unit (e.g., DistanceUnit.INCH).
     */
    public void setDistanceUnit(DistanceUnit unit) {
        this.distanceUnit = unit;
    }

    /**
     * Gets the current distance unit.
     * @return The currently configured DistanceUnit.
     */
    public DistanceUnit getDistanceUnit() {
        return this.distanceUnit;
    }

    /**
     * Sets the angle unit for integrated gyroscope measurements.
     * @param unit The desired angle unit (e.g., AngleUnit.DEGREES).
     */
    public void setAngleUnit(AngleUnit unit) {
        this.angleUnit = unit;
    }

    /**
     * Gets the current angle unit.
     * @return The currently configured AngleUnit.
     */
    public AngleUnit getAngleUnit() {
        return this.angleUnit;
    }

    /**
     * Sets the physical orientation of the sensor to ensure readings are correctly
     * transformed into the robot's coordinate frame.
     * @param rotation The physical rotation of the sensor.
     */
    public void setSensorRotation(Rotation rotation) {
        this.rotation = rotation;
    }

    /**
     * Gets the current sensor orientation.
     * @return The currently configured Rotation.
     */
    public Rotation getSensorRotation() {
        return this.rotation;
    }

    /**
     * Reads the raw 26-byte integral frame from the sensor.
     * The PX4Flow protocol uses a combined write-read operation.
     * @return A byte array containing the raw sensor data.
     */
    protected byte[] readRawIntegralFrame() {
        return deviceClient.read(REG_READ_INTEGRAL_FRAME, INTEGRAL_FRAME_LENGTH);
    }

    /**
     * Fetches the latest integrated data frame from the sensor.
     * This method performs a fresh I2C read and returns a fully parsed, converted,
     * and rotated data frame. This should be the primary method for polling the sensor.
     *
     * @return An {@link IntegralFrame} object containing the latest sensor data.
     */
    public IntegralFrame getIntegralFrame() {
        byte[] rawData = readRawIntegralFrame();
        ByteBuffer buffer = ByteBuffer.wrap(rawData);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        IntegralFrame frame = new IntegralFrame();

        // --- Parse raw little-endian values from the byte buffer ---
        // Unsigned values are masked to prevent sign extension in Java.
        frame.frameCountSinceLastReadout = buffer.getShort(0) & 0xFFFF; // uint16_t
        float px_flow_x_rad = (float) (buffer.getShort(2) / INTEGRAL_VALUE_TO_RADIANS);
        float px_flow_y_rad = (float) (buffer.getShort(4) / INTEGRAL_VALUE_TO_RADIANS);
        float gyro_x_rad = (float) (buffer.getShort(6) / INTEGRAL_VALUE_TO_RADIANS);
        float gyro_y_rad = (float) (buffer.getShort(8) / INTEGRAL_VALUE_TO_RADIANS);
        float gyro_z_rad = (float) (buffer.getShort(10) / INTEGRAL_VALUE_TO_RADIANS);
        frame.integrationTimespanMicroseconds = buffer.getInt(12) & 0xFFFFFFFFL; // uint32_t
        frame.sonarTimestampMicroseconds = buffer.getInt(16) & 0xFFFFFFFFL; // uint32_t
        float ground_dist_meters = (float) (buffer.getShort(20) / RAW_DISTANCE_TO_METERS);
        frame.gyroTemperatureCelsius = (float) (buffer.getShort(22) / RAW_TEMP_TO_CELSIUS);
        frame.quality = buffer.get(24) & 0xFF; // uint8_t

        // --- Apply sensor rotation to measurements ---
        float[] rotatedFlow = rotate(this.rotation, px_flow_x_rad, px_flow_y_rad, 0f);
        frame.pixelFlowXIntegral = rotatedFlow[0];
        frame.pixelFlowYIntegral = rotatedFlow[1];

        float[] rotatedGyro = rotate(this.rotation, gyro_x_rad, gyro_y_rad, gyro_z_rad);

        // --- Convert to user-specified units ---
        frame.groundDistance = (float) distanceUnit.fromMeters(ground_dist_meters);
        frame.gyroXRateIntegral = (float) angleUnit.fromRadians(rotatedGyro[0]);
        frame.gyroYRateIntegral = (float) angleUnit.fromRadians(rotatedGyro[1]);
        frame.gyroZRateIntegral = (float) angleUnit.fromRadians(rotatedGyro[2]);

        return frame;
    }

    /**
     * Helper method to rotate a 3-axis vector based on the sensor's physical orientation.
     * This logic is ported from the PX4 C++ driver.
     * @param rot The target rotation.
     * @param x The value on the X-axis.
     * @param y The value on the Y-axis.
     * @param z The value on the Z-axis.
     * @return A 3-element float array containing the rotated {x, y, z} values.
     */
    private float[] rotate(Rotation rot, float x, float y, float z) {
        float tmp;
        switch (rot) {
            case ROTATION_NONE:
                // No change
                break;
            case ROTATION_YAW_90:
                tmp = x; x = -y; y = tmp;
                break;
            case ROTATION_YAW_180:
                x = -x; y = -y;
                break;
            case ROTATION_YAW_270:
                tmp = x; x = y; y = -tmp;
                break;
            case ROTATION_ROLL_180:
                y = -y; z = -z;
                break;
            case ROTATION_ROLL_180_YAW_90:
                tmp = x; x = y; y = tmp; z = -z;
                break;
            case ROTATION_PITCH_180:
                x = -x; z = -z;
                break;
            case ROTATION_ROLL_180_YAW_270:
                tmp = x; x = -y; y = -tmp; z = -z;
                break;
            case ROTATION_ROLL_90:
                tmp = y; y = z; z = -tmp;
                break;
            case ROTATION_ROLL_270:
                tmp = y; y = -z; z = tmp;
                break;
            case ROTATION_PITCH_90:
                tmp = x; x = z; z = -tmp;
                break;
            case ROTATION_PITCH_270:
                tmp = x; x = -z; z = tmp;
                break;
            case ROTATION_DOWNWARD_FACING:
                // Common orientation for PX4Flow on rovers/drones
                tmp = x; x = y; y = -tmp;
                break;
            // Additional complex rotations can be added here if needed
            default:
                // Default to no rotation for unhandled cases
                break;
        }
        return new float[]{x, y, z};
    }
}