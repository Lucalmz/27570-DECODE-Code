package org.firstinspires.ftc.teamcode.pedroPathing.Services;

import static org.firstinspires.ftc.teamcode.pedroPathing.library.ConstantLib.FILENAME;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class IOStream {
    /**
     * 从文件中读取字符串
     *
     * @return 返回文件中的字符串内容，如果文件不存在或读取失败则返回默认值
     */
    public static String readSequenceFromFile() throws IOException {
        File file = new File(AppUtil.ROOT_FOLDER, FILENAME);
        BufferedReader reader = new BufferedReader(new FileReader(file) );
        String line = reader.readLine();
        reader.close();
        if (line == null) {
            throw new IOException();
        }
        return line;
    }
}
