package com.example.sample12;

import android.Manifest;
import android.app.AlarmManager;
import android.app.PendingIntent;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.Address;
import android.location.Geocoder;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.telephony.SmsManager;
import android.util.Log;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.UUID;


public class MainActivity extends AppCompatActivity
{
    //
    public String name="";

    public String id="";

//    public String age ="";

    private String nowlocation = "";

    public int count = 0;

    private static boolean exec = false;

    private final int REQUEST_BLUETOOTH_ENABLE = 100;

    private TextView mConnectionStatus;
    private Button btn_pairing;

    ConnectedTask mConnectedTask = null;
    static BluetoothAdapter mBluetoothAdapter;
    private String mConnectedDeviceName = null;
    private ArrayAdapter<String> mConversationArrayAdapter;
    static boolean isConnectionError = false;
    private static final String TAG = "BluetoothClient";

    ImageButton btn_call119,btn_hospital;
    TextView textName;



    private final int PERMISSION_ACCESS_FINE_LOCATION =1000;
    private final int PERMISSION_ACCESS_COARSE_LOCATION =1000;
    private boolean isAccessFineLocation = false;
    private boolean isAccessCoarseLocation = false;
    private boolean isPermission = false;


    public GpsTracker gps2;



    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);



        btn_call119 = (ImageButton) findViewById(R.id.btn_call119);
        btn_hospital = (ImageButton) findViewById(R.id.btn_hospital);

        textName = findViewById(R.id.textName);

        Intent intent = getIntent();
        final String userName = intent.getStringExtra("userName");
//        final String userID = intent.getStringExtra("userID");
//        final String userAge = intent.getStringExtra("userAge");
        textName.setText(userName);
        name = userName;
//        id=userID;
//        age = userAge;



        btn_call119.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View view)
            {
                Intent k = new Intent(MainActivity.this, AlarmActivity.class);
                k.putExtra("userID",id);
                startActivity(k);

                if(!isPermission){
                    callPermission();
                    return;
                }

                gps2 = new GpsTracker(MainActivity.this);


                double latitude = gps2.getLatitude();  //위도
                double longitude = gps2.getLongitude(); //경도

                nowlocation = getCurrentAddress(latitude,longitude);
                SendSMS("01088067574", userName+"님에게 응급상황이 발생하였습니다. 신속하게 안전을 확인해 주세요" +
                        "\n"+"사고 발생 주소: " +nowlocation+"\n"+"환자 인적 사항: "+userName+",");
                Toast.makeText(getApplicationContext(),"낙상발생! 119에 신고합니다.",Toast.LENGTH_LONG).show();
                SendSMS("01049365174", userName+"님에게 응급상황이 발생하였습니다. 신속하게 안전을 확인해 주세요" +
                        "\n"+"사고 발생 주소: " +nowlocation+"\n"+"환자 인적 사항: "+userName+",");
                Toast.makeText(getApplicationContext(),"낙상발생! 119에 신고합니다.",Toast.LENGTH_LONG).show();

            }
        });

        callPermission();
/////////////gps INFO//////////

        mConnectionStatus = (TextView)findViewById(R.id.connection_status_textview);

        mConversationArrayAdapter = new ArrayAdapter<>( this,
                android.R.layout.simple_list_item_1 );
        //  mMessageListview.setAdapter(mConversationArrayAdapter);


        Log.d( TAG, "Initalizing Bluetooth adapter...");

        //블루투스 관련된 모든 작업은 이 함수를 이용하 BluetoothAdapter 클래를 얻는 작업부터 시작
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        btn_pairing = findViewById(R.id.btn_pairing);
        btn_pairing.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {
                if (mBluetoothAdapter == null) {  //null을 반환하면 스마트폰에서 블루투스 지원을 하지 않는 것
                    showErrorDialog("This device is not implement Bluetooth.");
                    return;
                }
                // 블루투스가 활성상태인지 파악
                if (!mBluetoothAdapter.isEnabled()) { // 활성상태 아니라면 이 코드로 활성상태로 변경 가능
                    Intent intent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                    startActivityForResult(intent, REQUEST_BLUETOOTH_ENABLE);
                }
                else {
                    Log.d(TAG, "Initialisation successful.");

                    showPairedDevicesListDialog();
                }
            }
        });




        SetListener();





    }

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
            Toast.makeText(this, "주소 미발견", Toast.LENGTH_LONG).show();
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



    /////////////

    public void SetListener()
    {


        btn_hospital.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View view)
            {
                Intent intent = new Intent(MainActivity.this, SearchHospital.class);
                startActivity(intent);
            }
        });
    }



    public void SendSMS(String number, String msg){
        SmsManager sms = SmsManager.getDefault();
        sms.sendTextMessage(number,null, msg,null,null);
    }


    @Override
    protected void onDestroy() {
        super.onDestroy();

        if ( mConnectedTask != null ) {

            mConnectedTask.cancel(true);
        }
    }


    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        switch (requestCode) {
            // 블루투스 활성화
            case REQUEST_BLUETOOTH_ENABLE:
                if (resultCode == RESULT_OK) {
                    //BlueTooth is now Enabled
                    showPairedDevicesListDialog();
                }
                if (resultCode == RESULT_CANCELED) {
                    showQuitDialog("You need to enable bluetooth");
                }
                break;
        }
    }

    private class ConnectTask extends AsyncTask<Void, Void, Boolean> {

        private BluetoothSocket mBluetoothSocket = null;
        private BluetoothDevice mBluetoothDevice = null;

        ConnectTask(BluetoothDevice bluetoothDevice) {
            mBluetoothDevice = bluetoothDevice;
            mConnectedDeviceName = bluetoothDevice.getName();

            //SPP
            UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");


            try {
                mBluetoothSocket = mBluetoothDevice.createRfcommSocketToServiceRecord(uuid);
                Log.d( TAG, "create socket for "+mConnectedDeviceName);

            } catch (IOException e) {
                Log.e( TAG, "socket create failed " + e.getMessage());
            }

            mConnectionStatus.setText("connecting...");
        }


        @Override
        protected Boolean doInBackground(Void... params) {

            // Always cancel discovery because it will slow down a connection
            mBluetoothAdapter.cancelDiscovery();

            // Make a connection to the BluetoothSocket
            try {
                // This is a blocking call and will only return on a
                // successful connection or an exception
                mBluetoothSocket.connect();
            } catch (IOException e) {
                // Close the socket
                try {
                    mBluetoothSocket.close();
                } catch (IOException e2) {
                    Log.e(TAG, "unable to close() " +
                            " socket during connection failure", e2);
                }

                return false;
            }

            return true;
        }


        @Override
        protected void onPostExecute(Boolean isSucess) {

            if ( isSucess ) {
                connected(mBluetoothSocket);
            }
            else{

                isConnectionError = true;
                Log.d( TAG,  "Unable to connect device");
                showErrorDialog("Unable to connect device");
            }
        }
    }


    public void connected( BluetoothSocket socket ) {
        mConnectedTask = new ConnectedTask(socket);
        mConnectedTask.execute();
    }



    private class ConnectedTask extends AsyncTask<Void, String, Boolean> {

        private InputStream mInputStream = null;
        private OutputStream mOutputStream = null;
        private BluetoothSocket mBluetoothSocket = null;

        ConnectedTask(BluetoothSocket socket){

            mBluetoothSocket = socket;
            try {  // 데이터 송, 수신 스트림을 얻어옴
                mInputStream = mBluetoothSocket.getInputStream();
                mOutputStream = mBluetoothSocket.getOutputStream();
            } catch (IOException e) {
                Log.e(TAG, "socket not created", e );
            }

            Log.d( TAG, "connected to "+mConnectedDeviceName);
            // mConnectionStatus.setText( "connected to "+mConnectedDeviceName);
            mConnectionStatus.setText( "연결 되었습니다.");
        }


        // @RequiresApi(api = Build.VERSION_CODES.M)
        @Override
        protected Boolean doInBackground(Void... params) {

            byte [] readBuffer = new byte[1024];
            int readBufferPosition = 0;


            while (true) {
                if ( isCancelled() ) return false;
                try {
                    int bytesAvailable = mInputStream.available();
                    if(bytesAvailable > 0) {
                        setAlarm();


                        byte[] packetBytes = new byte[bytesAvailable];
                        mInputStream.read(packetBytes);
                        for(int i=0;i<bytesAvailable;i++) {

                            byte b = packetBytes[i];
                            if(b == '\n')
                            {
                                byte[] encodedBytes = new byte[readBufferPosition];
                                System.arraycopy(readBuffer, 0, encodedBytes, 0,
                                        encodedBytes.length);
                                String recvMessage = new String(encodedBytes, "UTF-8");

                                readBufferPosition = 0;

                                Log.d(TAG, "recv message: " + recvMessage);
                                publishProgress(recvMessage);
                            }
                            else
                            {
                                readBuffer[readBufferPosition++] = b;
                            }
                        }
                    }
                } catch (IOException e) {

                    Log.e(TAG, "disconnected", e);
                    return false;
                }
            }

        }

        @Override
        protected void onProgressUpdate(String... recvMessage) {

            mConversationArrayAdapter.insert(mConnectedDeviceName + ": " + recvMessage[0], 0);
        }

        @Override
        protected void onPostExecute(Boolean isSucess) {
            super.onPostExecute(isSucess);

            if ( !isSucess ) {


                closeSocket();
                Log.d(TAG, "Device connection was lost");
                isConnectionError = true;
                showErrorDialog("Device connection was lost");
            }
        }

        @Override
        protected void onCancelled(Boolean aBoolean) {
            super.onCancelled(aBoolean);

            closeSocket();
        }

        void closeSocket(){

            try {

                mBluetoothSocket.close();
                Log.d(TAG, "close socket()");

            } catch (IOException e2) {

                Log.e(TAG, "unable to close() " +
                        " socket during connection failure", e2);
            }
        }

//        void write(String msg){
//
//            msg += "\n";
//
//            try {
//                mOutputStream.write(msg.getBytes());
//                mOutputStream.flush();
//            } catch (IOException e) {
//                Log.e(TAG, "Exception during send", e );
//            }
//
//            //mInputEditText.setText(" ");
//        }
    }


    public void showPairedDevicesListDialog()
    {
        Set<BluetoothDevice> devices = mBluetoothAdapter.getBondedDevices();
        final BluetoothDevice[] pairedDevices = devices.toArray(new BluetoothDevice[0]);

        if ( pairedDevices.length == 0 ){
            showQuitDialog( "No devices have been paired.\n"
                    +"You must pair it with another device.");
            return;
        }

        String[] items;
        items = new String[pairedDevices.length];
        for (int i=0;i<pairedDevices.length;i++) {
            items[i] = pairedDevices[i].getName();
        }

        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("Select device");
        builder.setCancelable(false);
        builder.setItems(items, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();

                ConnectTask task = new ConnectTask(pairedDevices[which]);
                task.execute();
            }
        });
        builder.create().show();
    }



    public void showErrorDialog(String message)
    {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("Quit");
        builder.setCancelable(false);
        builder.setMessage(message);
        builder.setPositiveButton("OK",  new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
                if ( isConnectionError  ) {
                    isConnectionError = false;
                    finish();
                }
            }
        });
        builder.create().show();
    }


    public void showQuitDialog(String message)
    {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("Quit");
        builder.setCancelable(false);
        builder.setMessage(message);
        builder.setPositiveButton("OK",  new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
                finish();
            }
        });
        builder.create().show();
    }






    //알람 함수
    //  @RequiresApi(api = Build.VERSION_CODES.M)

    private void setAlarm() {

        // Receiver 설정
        Intent intent = new Intent(this, AlarmReceiver.class);
        PendingIntent pendingIntent = PendingIntent.getBroadcast(this, 0, intent, PendingIntent.FLAG_UPDATE_CURRENT);

        // 알람 설정
        AlarmManager alarmManager = (AlarmManager) getSystemService(ALARM_SERVICE);
        // alarmManager.set(AlarmManager.RTC_WAKEUP, this.calendar.getTimeInMillis(), pendingIntent);
        alarmManager.set(AlarmManager.RTC_WAKEUP, 0, pendingIntent);
//
//        // Toast 보여주기 (알람 시간 표시)
//        SimpleDateFormat format = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault());
//        Toast.makeText(this, "Alarm : " + format.format(calendar.getTime()), Toast.LENGTH_LONG).show();


    }


}