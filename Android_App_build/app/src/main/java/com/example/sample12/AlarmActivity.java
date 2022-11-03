package com.example.sample12;

import android.Manifest;
import android.content.pm.PackageManager;
import android.location.Address;
import android.location.Geocoder;
import android.media.MediaPlayer;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.telephony.SmsManager;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import java.io.IOException;
import java.util.List;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;

public class AlarmActivity extends AppCompatActivity {


    private int counter = 10;
    private TextView timer_text;
    private final Handler handler = new Handler();

    private MediaPlayer mediaPlayer;

    private Timer timer = new Timer();

    private String nowlo = "";



    private final int PERMISSION_ACCESS_FINE_LOCATION =1000;
    private final int PERMISSION_ACCESS_COARSE_LOCATION =1000;
    private boolean isAccessFineLocation = false;
    private boolean isAccessCoarseLocation = false;
    private boolean isPermission = false;

    public GpsTracker gps;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_alarm);
        timer_text= findViewById(R.id.timer);



        gps = new GpsTracker(AlarmActivity.this);

        double latitude2 = gps.getLatitude();
        double longitude2 = gps.getLongitude();


        nowlo = getCurrentAddress(latitude2,longitude2);

        //시간측정
        TimerTask tt = new TimerTask() {
            @Override
            public void run() {
                Update();
                counter--;
                if(counter == 0){
                    counter = 10;

                    if(!isPermission){
                        callPermission();
                        return;
                    }

                    SendSMS("01049365174", nowlo + "\n" + "낙상사고 발생");
                    SendSMS("01088067574", nowlo + "\n" + "낙상사고 발생");
                    StopTimer();
                    finish();
                }
            }
        };

        callPermission();
        //타이머 생성
        timer.schedule(tt,0,1000);

        // 알람음 재생
        this.mediaPlayer = MediaPlayer.create(this, R.raw.alarm);
        this.mediaPlayer.setLooping(true);
        this.mediaPlayer.start();

        findViewById(R.id.btnClose).setOnClickListener(mClickListener);
    }

    public void StopTimer(){
        timer.cancel();
    }

    //문자보내기함수
    public void SendSMS(String number, String msg){
        SmsManager sms = SmsManager.getDefault();
        sms.sendTextMessage(number,null, msg,null,null);
    }
    //알람업데이트 시간
    protected void Update() {
        Runnable updater = new Runnable() {
            public void run() {
                timer_text.setText(counter + "초");
            }
        };
        handler.post(updater);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        // MediaPlayer release
        if (this.mediaPlayer != null) {
            this.mediaPlayer.release();
            this.mediaPlayer = null;
        }
    }

    /* 알람 종료 */
    private void close() {
        if (this.mediaPlayer.isPlaying()) {
            this.mediaPlayer.stop();
            this.mediaPlayer.release();
            this.mediaPlayer = null;
        }

        finish();
    }

    View.OnClickListener mClickListener = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            switch (v.getId()) {
                case R.id.btnClose:
                    // 알람 종료
                    close();
                    StopTimer();
                    finish();
                    break;
            }
        }
    };


    public String getCurrentAddress( double latitude, double longitude) {

        //지오코더... GPS를 주소로 변환
        Geocoder geocoder = new Geocoder(this, Locale.getDefault());

        List<Address> addresses;

        try {

            addresses = geocoder.getFromLocation(
                    latitude,
                    longitude,
                    7);
        } catch (IOException ioException) {
            //네트워크 문제
            Toast.makeText(this, "지오코더 서비스 사용불가", Toast.LENGTH_LONG).show();
            return "지오코더 서비스 사용불가";
        } catch (IllegalArgumentException illegalArgumentException) {
            Toast.makeText(this, "잘못된 GPS 좌표", Toast.LENGTH_LONG).show();
            return "잘못된 GPS 좌표";

        }



        if (addresses == null || addresses.size() == 0) {
//            Toast.makeText(this, "주소 미발견", Toast.LENGTH_LONG).show();
            return "주소 미발견";

        }

        Address address = addresses.get(0);
        return address.getAddressLine(0).toString()+"\n";

    }


    public void onRequestPermissionResult(int requestCode, String[] permission, int[] grantResult){
        if(requestCode == PERMISSION_ACCESS_FINE_LOCATION && grantResult[0] == PackageManager.PERMISSION_GRANTED){
            isAccessFineLocation = true;
        }else if (requestCode == PERMISSION_ACCESS_COARSE_LOCATION && grantResult[0] == PackageManager.PERMISSION_GRANTED){
            isAccessCoarseLocation = true;
        }

        if(isAccessFineLocation && isAccessCoarseLocation){
            isPermission= true;
        }
    }

    //전화번호 권한 요청
    public void callPermission(){
        if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.M && checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION)!= PackageManager.PERMISSION_GRANTED){
            requestPermissions( new String[]{Manifest.permission.ACCESS_FINE_LOCATION},PERMISSION_ACCESS_FINE_LOCATION);
        }else if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.M && checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION)!= PackageManager.PERMISSION_GRANTED){
            requestPermissions( new String[]{Manifest.permission.ACCESS_COARSE_LOCATION},PERMISSION_ACCESS_COARSE_LOCATION);
        }else{
            isPermission = true;
        }
    }

}