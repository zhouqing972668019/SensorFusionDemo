package com.zhouqing.chatproject.sensorfusiondemo;

import java.util.List;

public class Utils {

    public static void constructSensorResult(List<SensorInfo> sensorInfoList, List<ComparableSensorEvent> eventList){
        for(SensorInfo sensorInfo:sensorInfoList){
            float[] gyroAngle = getSensorInfo(eventList,sensorInfo.timeStamp,"gyro");
            float[] magAccAngle = getSensorInfo(eventList,sensorInfo.timeStamp,"mag_acc");
            sensorInfo.angle_y_gyro = gyroAngle[2];
            sensorInfo.angle_y_mag_acc = magAccAngle[0];
        }

    }

    public static float[] getSensorInfo(List<ComparableSensorEvent> eventList,long targetTimeStamp,String targetType){
        float[] result = new float[3];
        for(ComparableSensorEvent event:eventList){
            String type = event.type;
            if(type.equals(targetType))
            {
                long timestamp = event.timestamp;
                if(targetTimeStamp <= timestamp)
                {
                    for(int i=0;i<3;i++){
                        result[i] = (result[i]+event.values[i])/2f;
                    }
                    break;
                }
                for(int i=0;i<3;i++){
                    result[i] = event.values[i];
                }
            }
        }
        return result;
    }
}
