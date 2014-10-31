package com.realwakka.indoor;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.TextView;

import java.util.Timer;
import java.util.TimerTask;


public class MyActivity extends Activity{
    TextView mTextView;
    SensorManager mSensorManager;
    SensorFusionListener mListener;

    Sensor mAccelerometer;
    Sensor mGyroscope;
    Sensor mMagnetic;
    Sensor mLinear;

    float[] RotationMatrix = new float[9];
    float[] Inclination = new float[9];

    float[] GyroscopeData = new float[3];
    float[] MagneticData = new float[3];
    float[] AccelerometerData = new float[3];
    float[] LinearData = new float[3];

    boolean AccelerometerSet = false;
    boolean GyroscopeSet = false;
    boolean MagneticSet = false;
    boolean LinearSet = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_my);

        mTextView = (TextView) findViewById(R.id.my_textView);

        mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);


        mListener = new SensorFusionListener();
        initListeners();
    }

    public void initListeners(){
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mSensorManager.registerListener(mListener, mAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);

        mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mSensorManager.registerListener(mListener, mGyroscope, SensorManager.SENSOR_DELAY_FASTEST);

        mMagnetic = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mSensorManager.registerListener(mListener, mMagnetic,SensorManager.SENSOR_DELAY_FASTEST);

        mLinear = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        mSensorManager.registerListener(mListener,mLinear,SensorManager.SENSOR_DELAY_FASTEST);
    }

    class SensorFusionListener implements SensorEventListener{
        @Override
        public void onSensorChanged(SensorEvent event) {
            if(event.sensor==mAccelerometer){
                System.arraycopy(event.values,0,AccelerometerData,0,3);
                AccelerometerSet = true;
            }else if(event.sensor==mGyroscope){
                System.arraycopy(event.values,0,GyroscopeData,0,3);
                GyroscopeSet = true;
            }else if(event.sensor==mMagnetic){
                System.arraycopy(event.values,0,MagneticData,0,3);
                MagneticSet = true;
            }else{

            }
            if(AccelerometerSet && MagneticSet) {
                SensorManager.getRotationMatrix(RotationMatrix, null, AccelerometerData, MagneticData);

                float[] orientation = new float[3];
                SensorManager.getOrientation(RotationMatrix,orientation);

                String str = "x : "+orientation[0]+"\ny : "+orientation[1]+"\nz : "+orientation[2];
                mTextView.setText(str);


            }


        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    }
}
