package org.firstinspires.ftc.teamcode.pedroPathing.Services;
import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Properties;

/**
 * 一个用于在Control Hub上持久化存储和读取键值对数据的服务。
 * 数据被保存在一个属性文件中，方便进行简单的配置管理。
 *
 * 使用方法:
 * 1. init()方法中初始化它:
 *    configService = new ConfigurationService(hardwareMap.appContext);
 *
 * 2. 保存数据:
 *    configService.saveData("allianceColor", "BLUE");
 *
 * 3. 读取数据:
 *    String color = configService.getData("allianceColor", "RED"); // "RED"是找不到时的默认值
 */
public class IOStream {

    private static final String TAG = "RunSettingInformation";
    private static final String CONFIG_FILE_NAME = "robot_settings.properties";

    private final Context appContext;
    private final Properties properties;
    private final File configFile;

    /**
     * 构造函数，需要传入Android应用的Context。
     *
     * @param context 通常从 hardwareMap.appContext 获取。
     */
    public IOStream(Context context) {
        this.appContext = context;
        this.properties = new Properties();
        // getFilesDir() 会获取应用的内部存储目录，这个目录是应用私有的
        this.configFile = new File(appContext.getFilesDir(), CONFIG_FILE_NAME);
        loadProperties();
    }

    /**
     * 保存一个键值对。
     *
     * @param key   要保存的数据的键（例如："allianceColor"）。
     * @param value 要保存的数据的值（例如："BLUE"）。
     */
    public void saveData(String key, String value) {
        properties.setProperty(key, value);
        try (FileOutputStream fos = new FileOutputStream(configFile)) {
            // 将Properties对象写入文件。第二个参数是注释，可以为null。
            properties.store(fos, "Robot Configuration");
            RobotLog.i(TAG, "Saved data: " + key + " = " + value);
        } catch (IOException e) {
            RobotLog.e(TAG, "Error saving data to " + configFile.getAbsolutePath(), e);
        }
    }

    /**
     * 根据键来获取其对应的值。
     *
     * @param key 要查询的键。
     * @return 如果键存在，则返回其对应的值；如果不存在，则返回 null。
     */
    public String getData(String key) {
        return properties.getProperty(key);
    }

    /**
     * 根据键来获取其对应的值，如果键不存在，则返回一个默认值。
     *
     * @param key          要查询的键。
     * @param defaultValue 如果找不到键，则返回此默认值。
     * @return 键对应的值或提供的默认值。
     */
    public String getData(String key, String defaultValue) {
        return properties.getProperty(key, defaultValue);
    }

    /**
     * 从文件中加载所有属性。
     * 在服务初始化时会自动调用。
     */
    private void loadProperties() {
        if (configFile.exists()) {
            try (FileInputStream fis = new FileInputStream(configFile)) {
                properties.load(fis);
                RobotLog.i(TAG, "Successfully loaded properties from " + configFile.getAbsolutePath());
            } catch (IOException e) {
                RobotLog.e(TAG, "Error loading data from " + configFile.getAbsolutePath(), e);
            }
        } else {
            RobotLog.w(TAG, "Config file not found. A new one will be created on first save.");
        }
    }
}