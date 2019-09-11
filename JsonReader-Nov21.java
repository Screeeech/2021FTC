package org.firstinspires.ftc.teamcode.util.JsonReaders;

import com.qualcomm.robotcore.util.RobotLog;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Iterator;


/*
 * Copyright (c) 2016 Robocracy 9773
 */

public class JsonReader {
    public static final String configFile = new String("myconfig.json");


    private String jsonFilePath;
    public String jsonStr;
    public JSONObject jsonRoot;

    public JsonReader(String filePath) {
        FileReader fileReader = null;
        BufferedReader bufReader = null;
        StringBuilder strBuilder = new StringBuilder();
        String line = null;
        // If the given file path does not exist, give an error
        try {
            fileReader = new FileReader(filePath);
            bufReader = new BufferedReader(fileReader);
        }
        catch (IOException except) {
            RobotLog.ee("ftcFMHS: Error while trying to open the json file %s", filePath);
            RobotLog.ee("ftcFMHS: %s", except.getMessage());
        }

        // Read the file and append to the string builder
        try {
            while ((line = bufReader.readLine()) != null) {
                strBuilder.append(line);
            }
            // Now initialize the main variable that holds the entire json config
            jsonStr = new String(strBuilder);
        }
        catch (IOException except) {
            RobotLog.ee("ftpFMHS: Error while reading the json file %s", filePath);
            RobotLog.ee("ftpFMHS: %s", except.getMessage());
        }
        try {
            jsonRoot = new JSONObject(jsonStr);
        }
        catch (JSONException except) {
            RobotLog.ee("ftpFMHS: Error while parsing the json file.  Error message = %s",
                    except.getMessage());
        }
        return;
    }

    // This is a class method
    public static String getRealKeyIgnoreCase(JSONObject jobj, String key) throws JSONException {
        Iterator<String> iter = jobj.keys();
        while (iter.hasNext()) {
            String key1 = iter.next();
            if (key1.equalsIgnoreCase(key)) {
                return (key1);
            }
        }
        return null;
    }


}
