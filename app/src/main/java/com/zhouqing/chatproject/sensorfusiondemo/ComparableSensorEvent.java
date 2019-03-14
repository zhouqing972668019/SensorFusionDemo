package com.zhouqing.chatproject.sensorfusiondemo;

public class ComparableSensorEvent {
    public float[] values ;
    public String type;
    public long timestamp;

    public ComparableSensorEvent(float[] values, long timeString, String type) {
        this.values  = values.clone();
        this.type = type;
        this.timestamp = timeString;
    }
}
