package com.realwakka.indoor;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.Matrix;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.TextView;

import java.util.Timer;
import java.util.TimerTask;


public class MyActivity extends Activity{
    TextView mTextView;
    TextView mVelocityView;

    SensorManager mSensorManager;
    SensorFusionListener mListener;

    Sensor mAccelerometer;
    Sensor mGyroscope;
    Sensor mMagnetic;
    Sensor mLinear;

    float[] RotationMatrix = new float[9];
    float[] RotationMatrix2 = new float[16];


    float[] Inclination = new float[9];

    float[] GyroscopeData = new float[3];
    float[] MagneticData = new float[3];
    float[] AccelerometerData = new float[3];
    float[] LinearData = new float[3];

    float[] Velocity = new float[3];

    boolean AccelerometerSet = false;
    boolean GyroscopeSet = false;
    boolean MagneticSet = false;
    boolean LinearSet = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_my);

        mTextView = (TextView) findViewById(R.id.my_textView);
        mVelocityView = (TextView) findViewById(R.id.my_velocity);

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
    private float[] getTrueAcceleration(float[] accelerometervalues, float[] orientationvalues){
        float[] trueacceleration = new float[3];
        trueacceleration[0] =(float) (accelerometervalues[0]*(Math.cos(orientationvalues[2])*Math.cos(orientationvalues[0])+Math.sin(orientationvalues[2])*Math.sin(orientationvalues[1])*Math.sin(orientationvalues[0])) + accelerometervalues[1]*(Math.cos(orientationvalues[1])*Math.sin(orientationvalues[0])) + accelerometervalues[2]*(-Math.sin(orientationvalues[2])*Math.cos(orientationvalues[0])+Math.cos(orientationvalues[2])*Math.sin(orientationvalues[1])*Math.sin(orientationvalues[0])));
        trueacceleration[1] = (float) (accelerometervalues[0]*(-Math.cos(orientationvalues[2])*Math.sin(orientationvalues[0])+Math.sin(orientationvalues[2])*Math.sin(orientationvalues[1])*Math.cos(orientationvalues[0])) + accelerometervalues[1]*(Math.cos(orientationvalues[1])*Math.cos(orientationvalues[0])) + accelerometervalues[2]*(Math.sin(orientationvalues[2])*Math.sin(orientationvalues[0])+ Math.cos(orientationvalues[2])*Math.sin(orientationvalues[1])*Math.cos(orientationvalues[0])));
        trueacceleration[2] = (float) (accelerometervalues[0]*(Math.sin(orientationvalues[2])*Math.cos(orientationvalues[1])) + accelerometervalues[1]*(-Math.sin(orientationvalues[1])) + accelerometervalues[2]*(Math.cos(orientationvalues[2])*Math.cos(orientationvalues[1])));
        return trueacceleration;
    }


    public void onClick(View v){
        switch(v.getId()){
            case R.id.my_reset:
                Velocity[0]=0f;
                Velocity[1]=0f;
                Velocity[2]=0f;
                break;

        }
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
            }else if(event.sensor==mMagnetic) {
                System.arraycopy(event.values, 0, MagneticData, 0, 3);
                MagneticSet = true;
            }else if(event.sensor==mLinear){
                    System.arraycopy(event.values,0,LinearData,0,3);
                    LinearSet = true;

            }else{

            }
            if(AccelerometerSet && MagneticSet && LinearSet) {
                SensorManager.getRotationMatrix(RotationMatrix, null, AccelerometerData, MagneticData);
                SensorManager.getRotationMatrix(RotationMatrix2, null, AccelerometerData, MagneticData);

                float[] inv = new float[16];
                float[] result = new float[4];
                float[] linear2 = new float[4];

                linear2[0] = LinearData[0];
                linear2[1] = LinearData[1];
                linear2[2] = LinearData[2];

                Matrix.invertM(inv,0,RotationMatrix2,0);

                Matrix.multiplyMV(result,0,inv,0,linear2,0);

                float[] orientation = new float[3];
                float[] orientation2 = new float[4];
                SensorManager.getOrientation(RotationMatrix,orientation);
                SensorManager.getOrientation(RotationMatrix2,orientation2);

                float[] trueaccel = getTrueAcceleration(LinearData,orientation);

                String str = "x : "+result[0]+"\ny : "+result[1]+"\nz : "+(result[2]);

                Velocity[0] += result[0];
                Velocity[1] += result[1];
                Velocity[2] += result[2];

                float o_x = (float) Math.toDegrees(orientation2[0]);
                float o_y = (float) Math.toDegrees(orientation2[1]);
                float o_z = (float) Math.toDegrees(orientation2[2]);



                String v_str = "x : "+Velocity[0]+"\ny : "+Velocity[1]+"\nz : "+Velocity[2];
                v_str = v_str + "\nx : "+o_x+"\ny : "+o_y+"\nz : "+o_z;
                mTextView.setText(str);
                mVelocityView.setText(v_str);


            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    }
}
