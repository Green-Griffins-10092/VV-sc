package org.firstinspires.ftc.griffins.Testing;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;

import java.io.File;

/**
 * Created by David on 1/23/2017.
 * class for easy file output for testing purposes
 */

public class FileOutput {
    static void outputFile(String saveFile, String outputData) {
        File file = AppUtil.getInstance().getSettingsFile(saveFile);
        ReadWriteFile.writeFile(file, outputData);
    }
}
