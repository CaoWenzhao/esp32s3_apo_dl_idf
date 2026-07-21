package com.smart.suitcase;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.content.Context;
import android.content.BroadcastReceiver;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.util.Log;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.GridLayout;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.Space;
import android.widget.TextView;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.ConsoleMessage;
import android.webkit.WebChromeClient;

import java.nio.charset.StandardCharsets;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayDeque;
import java.util.Locale;
import java.util.Queue;
import java.util.UUID;

public class MainActivity extends Activity {
    private static final String DEVICE_NAME = "SmartSuitcase";
    private static final UUID SERVICE_UUID = UUID.fromString("5f6d1000-8f3e-4c21-a7b2-3d4e5f607182");
    private static final UUID COMMAND_UUID = UUID.fromString("5f6d1001-8f3e-4c21-a7b2-3d4e5f607182");
    private static final UUID TELEMETRY_UUID = UUID.fromString("5f6d1002-8f3e-4c21-a7b2-3d4e5f607182");
    private static final UUID CCCD_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
    private static final int PERMISSION_REQUEST = 41;

    private static final int BG = Color.rgb(255, 255, 255);
    private static final int PANEL = Color.rgb(247, 248, 250);
    private static final int BORDER = Color.rgb(215, 220, 226);
    private static final int TEXT = Color.rgb(17, 19, 24);
    private static final int MUTED = Color.rgb(91, 101, 113);
    private static final int BLUE = Color.rgb(20, 113, 207);
    private static final int BLUE_DARK = Color.rgb(12, 91, 168);
    private static final int GREEN = Color.rgb(0, 139, 94);
    private static final int RED = Color.rgb(194, 24, 31);
    private static final int SENSOR_RED = Color.rgb(253, 235, 236);
    private static final int SENSOR_GREEN = Color.rgb(226, 246, 236);
    private static final int CONTROL = Color.rgb(232, 236, 241);

    private final Handler handler = new Handler(Looper.getMainLooper());
    private BluetoothAdapter adapter;
    private BluetoothLeScanner scanner;
    private BluetoothDevice pendingDevice;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic commandCharacteristic;
    private boolean scanning;
    private boolean ready;
    private boolean setupStarted;
    private boolean writeBusy;
    private final Queue<byte[]> writeQueue = new ArrayDeque<>();

    private TextView connectionText;
    private Button connectButton;
    private Button followButton;
    private Button manualButton;
    private TextView batteryValue;
    private TextView batterySub;
    private TextView weightValue;
    private TextView targetDistanceValue;
    private TextView distanceAlert;
    private TextView targetBearingValue;
    private TextView frontValue;
    private TextView sideValue;
    private TextView stateValue;
    private TextView motionValue;
    private TextView uwbBadge;
    private TextView lidarBadge;
    private TextView ultraLeftBadge;
    private TextView ultraRightBadge;
    private TextView fsrBadge;
    private TextView encoderBadge;
    private SeekBar followSpeed;
    private SeekBar followTurn;
    private SeekBar remoteSpeed;
    private TextView followSpeedLabel;
    private TextView followTurnLabel;
    private TextView remoteSpeedLabel;
    private final boolean[] sliderTouching = new boolean[3];
    private boolean manualMode;
    private boolean estop = true;

    private final BroadcastReceiver bondReceiver = new BroadcastReceiver() {
        @SuppressLint("MissingPermission")
        @Override
        public void onReceive(Context context, Intent intent) {
            if (!BluetoothDevice.ACTION_BOND_STATE_CHANGED.equals(intent.getAction())) return;
            BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
            if (device == null || pendingDevice == null ||
                    !device.getAddress().equals(pendingDevice.getAddress())) return;
            int state = intent.getIntExtra(BluetoothDevice.EXTRA_BOND_STATE, BluetoothDevice.BOND_NONE);
            int previous = intent.getIntExtra(BluetoothDevice.EXTRA_PREVIOUS_BOND_STATE,
                    BluetoothDevice.BOND_NONE);
            if (state == BluetoothDevice.BOND_BONDED) {
                setConnection("配对成功，正在连接…", false);
                if (gatt != null) {
                    beginGattSetup(gatt);
                } else {
                    connectDevice(device);
                }
            } else if (state == BluetoothDevice.BOND_NONE && previous != BluetoothDevice.BOND_NONE) {
                disconnectGatt();
                setConnection("已取消配对，设备已恢复广播，点连接重试", false);
            }
        }
    };

    private final Runnable scanTimeout = () -> {
        stopScan();
        setConnection("未找到设备，点此重试", false);
    };

    private final Runnable heartbeat = new Runnable() {
        @Override
        public void run() {
            if (ready) {
                if (writeQueue.size() < 6) {
                    sendCommand("H");
                }
                handler.postDelayed(this, 300);
            }
        }
    };

    private final Runnable speedSender = () -> sendCommand(String.format(
            Locale.US, "S,%d,%d,%d",
            followSpeed.getProgress(), followTurn.getProgress(), remoteSpeed.getProgress()));

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().setStatusBarColor(BG);
        getWindow().setNavigationBarColor(BG);
        getWindow().getDecorView().setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_LIGHT_STATUS_BAR | View.SYSTEM_UI_FLAG_LIGHT_NAVIGATION_BAR);
        IntentFilter bondFilter = new IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        if (Build.VERSION.SDK_INT >= 33) {
            registerReceiver(bondReceiver, bondFilter, Context.RECEIVER_NOT_EXPORTED);
        } else {
            registerReceiver(bondReceiver, bondFilter);
        }
        buildUi();
        BluetoothManager manager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        adapter = manager == null ? null : manager.getAdapter();
        ensurePermissionsAndScan();
    }

    @SuppressLint({"SetJavaScriptEnabled", "ClickableViewAccessibility"})
    private void buildUi() {
        ScrollView scroll = new ScrollView(this);
        scroll.setFillViewport(true);
        scroll.setBackgroundColor(BG);

        LinearLayout root = vertical();
        root.setPadding(dp(18), dp(18), dp(18), dp(32));
        scroll.addView(root, matchWrap());

        WebView modelView = new WebView(this);
        modelView.setBackgroundColor(Color.WHITE);
        modelView.setLayerType(View.LAYER_TYPE_HARDWARE, null);
        modelView.setVerticalScrollBarEnabled(false);
        modelView.setHorizontalScrollBarEnabled(false);
        WebSettings modelSettings = modelView.getSettings();
        modelSettings.setJavaScriptEnabled(true);
        modelSettings.setAllowFileAccess(true);
        modelSettings.setAllowContentAccess(true);
        modelSettings.setDomStorageEnabled(false);
        modelView.setWebChromeClient(new WebChromeClient() {
            @Override
            public boolean onConsoleMessage(ConsoleMessage message) {
                Log.d("Suitcase3D", message.message() + " @" + message.lineNumber());
                return true;
            }
        });
        modelView.setOnTouchListener((view, event) -> {
            boolean active = event.getActionMasked() != MotionEvent.ACTION_UP &&
                    event.getActionMasked() != MotionEvent.ACTION_CANCEL;
            view.getParent().requestDisallowInterceptTouchEvent(active);
            return false;
        });
        try {
            String modelHtml = readAsset("model/index.html");
            String threeJs = readAsset("model/three.min.js").replace("</script>", "<\\/script>");
            modelHtml = modelHtml.replace("<script src=\"three.min.js\"></script>",
                    "<script>" + threeJs + "</script>");
            modelView.loadDataWithBaseURL("https://appassets.androidplatform.net/model/",
                    modelHtml, "text/html", "UTF-8", null);
        } catch (IOException error) {
            Log.e("Suitcase3D", "Unable to load bundled model", error);
            modelView.loadData("<html><body style='font-family:sans-serif;text-align:center;padding-top:120px'>3D 模型资源加载失败</body></html>",
                    "text/html", "UTF-8");
        }
        root.addView(modelView, matchHeight(dp(340)));

        TextView productName = text("SmartSuitcase", 29, TEXT, true);
        productName.setGravity(Gravity.CENTER);
        root.addView(productName, withMargins(matchHeight(dp(52)), 0, 0, 0, dp(4)));

        LinearLayout header = horizontal();
        header.setGravity(Gravity.CENTER_VERTICAL);
        connectionText = text("正在准备蓝牙…", 14, MUTED, false);
        header.addView(connectionText, new LinearLayout.LayoutParams(0, dp(50), 1));
        connectButton = button("连接", BLUE, 15);
        connectButton.setOnClickListener(v -> {
            if (ready || gatt != null) {
                disconnectGatt();
            } else {
                ensurePermissionsAndScan();
            }
        });
        header.addView(connectButton, new LinearLayout.LayoutParams(dp(92), dp(46)));
        root.addView(header, withMargins(matchWrap(), 0, 0, 0, dp(12)));

        distanceAlert = text("距离警报", 18, Color.WHITE, true);
        distanceAlert.setGravity(Gravity.CENTER);
        distanceAlert.setBackground(round(RED, RED, 1));
        distanceAlert.setVisibility(View.GONE);
        root.addView(distanceAlert, withMargins(matchHeight(dp(58)), 0, 0, 0, dp(12)));

        Button stop = button("紧急停止", RED, 26);
        stop.setOnClickListener(v -> sendCommand("E"));
        root.addView(stop, withMargins(matchHeight(dp(92)), 0, 0, 0, dp(12)));

        LinearLayout armRow = horizontal();
        Button arm = button("解除急停 / ARM", GREEN, 17);
        arm.setOnClickListener(v -> sendCommand("A"));
        armRow.addView(arm, new LinearLayout.LayoutParams(0, dp(58), 1));
        stateValue = text("等待连接", 15, MUTED, true);
        stateValue.setGravity(Gravity.CENTER);
        stateValue.setBackground(round(PANEL, BORDER, 1));
        LinearLayout.LayoutParams stateLp = new LinearLayout.LayoutParams(0, dp(58), 1);
        stateLp.setMargins(dp(10), 0, 0, 0);
        armRow.addView(stateValue, stateLp);
        root.addView(armRow, withMargins(matchWrap(), 0, 0, 0, dp(12)));

        LinearLayout metrics1 = horizontal();
        Metric battery = metric("电池", "--%", "-- V");
        batteryValue = battery.value;
        batterySub = battery.sub;
        Metric weight = metric("重量", "-- kg", "压力传感器");
        weightValue = weight.value;
        addHalf(metrics1, battery.view, true);
        addHalf(metrics1, weight.view, false);
        root.addView(metrics1, withMargins(matchWrap(), 0, 0, 0, dp(10)));

        LinearLayout metrics2 = horizontal();
        Metric distance = metric("目标距离", "-- m", "UWB");
        targetDistanceValue = distance.value;
        Metric bearing = metric("目标方向", "--°", "左 + / 右 -");
        targetBearingValue = bearing.value;
        addHalf(metrics2, distance.view, true);
        addHalf(metrics2, bearing.view, false);
        root.addView(metrics2, withMargins(matchWrap(), 0, 0, 0, dp(10)));

        Metric front = metric("前向净空", "-- m", "仅激光雷达");
        frontValue = front.value;
        sideValue = front.sub;
        root.addView(front.view, withMargins(matchHeight(dp(126)), 0, 0, 0, dp(12)));

        LinearLayout modePanel = panel();
        LinearLayout modes = horizontal();
        followButton = button("跟随模式", BLUE_DARK, 18);
        manualButton = button("遥控模式", CONTROL, 18);
        manualButton.setTextColor(TEXT);
        followButton.setOnClickListener(v -> sendCommand("M,0"));
        manualButton.setOnClickListener(v -> sendCommand("M,1"));
        addHalf(modes, followButton, true);
        addHalf(modes, manualButton, false);
        modePanel.addView(modes, matchHeight(dp(62)));
        followSpeed = addSlider(modePanel, "跟随速度", 75, 0);
        followTurn = addSlider(modePanel, "跟随转向", 70, 1);
        remoteSpeed = addSlider(modePanel, "遥控速度", 80, 2);
        root.addView(modePanel, withMargins(matchWrap(), 0, 0, 0, dp(12)));

        LinearLayout controlPanel = panel();
        TextView manualTitle = text("遥控方向", 18, MUTED, true);
        controlPanel.addView(manualTitle, withMargins(matchWrap(), 0, 0, 0, dp(8)));
        GridLayout pad = new GridLayout(this);
        pad.setColumnCount(3);
        pad.setRowCount(3);
        pad.setAlignmentMode(GridLayout.ALIGN_BOUNDS);
        addPadSpace(pad);
        addDriveButton(pad, "▲", 100, 0, CONTROL);
        addPadSpace(pad);
        addDriveButton(pad, "◀", 0, 100, CONTROL);
        Button padStop = padButton("STOP", Color.rgb(130, 52, 55));
        padStop.setOnClickListener(v -> sendCommand("D,0,0"));
        pad.addView(padStop, padParams());
        addDriveButton(pad, "▶", 0, -100, CONTROL);
        addPadSpace(pad);
        addDriveButton(pad, "▼", -100, 0, CONTROL);
        addPadSpace(pad);
        controlPanel.addView(pad, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, dp(330)));
        root.addView(controlPanel, withMargins(matchWrap(), 0, 0, 0, dp(12)));

        LinearLayout sensorPanel = panel();
        TextView sensorTitle = text("传感器状态", 18, MUTED, true);
        sensorPanel.addView(sensorTitle, withMargins(matchWrap(), 0, 0, 0, dp(10)));
        LinearLayout sensorRow1 = horizontal();
        uwbBadge = sensorBadge("UWB");
        lidarBadge = sensorBadge("雷达");
        encoderBadge = sensorBadge("编码器");
        addThird(sensorRow1, uwbBadge, true);
        addThird(sensorRow1, lidarBadge, true);
        addThird(sensorRow1, encoderBadge, false);
        sensorPanel.addView(sensorRow1, matchHeight(dp(52)));
        LinearLayout sensorRow2 = horizontal();
        ultraLeftBadge = sensorBadge("左超声波");
        ultraRightBadge = sensorBadge("右超声波");
        fsrBadge = sensorBadge("压力");
        addThird(sensorRow2, ultraLeftBadge, true);
        addThird(sensorRow2, ultraRightBadge, true);
        addThird(sensorRow2, fsrBadge, false);
        LinearLayout.LayoutParams sensor2Lp = matchHeight(dp(52));
        sensor2Lp.setMargins(0, dp(8), 0, 0);
        sensorPanel.addView(sensorRow2, sensor2Lp);
        motionValue = text("速度 -- m/s  ·  转速 -- rad/s  ·  PWM -- / -- μs", 14, MUTED, false);
        motionValue.setPadding(0, dp(14), 0, 0);
        sensorPanel.addView(motionValue, matchWrap());
        root.addView(sensorPanel, matchWrap());

        setContentView(scroll);
    }

    private SeekBar addSlider(LinearLayout parent, String title, int initial, int index) {
        LinearLayout labelRow = horizontal();
        TextView name = text(title, 16, MUTED, false);
        TextView value = text(initial + "%", 16, TEXT, true);
        value.setGravity(Gravity.END);
        labelRow.addView(name, new LinearLayout.LayoutParams(0, dp(34), 1));
        labelRow.addView(value, new LinearLayout.LayoutParams(dp(72), dp(34)));
        LinearLayout.LayoutParams labelLp = matchWrap();
        labelLp.setMargins(0, dp(12), 0, 0);
        parent.addView(labelRow, labelLp);

        SeekBar seek = new SeekBar(this);
        seek.setMax(100);
        seek.setProgress(initial);
        seek.setProgressTintList(ColorStateList.valueOf(BLUE));
        seek.setThumbTintList(ColorStateList.valueOf(BLUE));
        seek.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                value.setText(String.format(Locale.CHINA, "%d%%", progress));
                if (fromUser) {
                    handler.removeCallbacks(speedSender);
                    handler.postDelayed(speedSender, 60);
                }
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
                sliderTouching[index] = true;
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                sliderTouching[index] = false;
                handler.removeCallbacks(speedSender);
                speedSender.run();
            }
        });
        parent.addView(seek, matchHeight(dp(36)));
        if (index == 0) followSpeedLabel = value;
        if (index == 1) followTurnLabel = value;
        if (index == 2) remoteSpeedLabel = value;
        return seek;
    }

    @SuppressLint("ClickableViewAccessibility")
    private void addDriveButton(GridLayout pad, String symbol, int linear, int angular, int color) {
        Button button = padButton(symbol, color);
        button.setLongClickable(true);
        button.setOnLongClickListener(v -> true);
        button.setOnTouchListener((v, event) -> {
            if (event.getActionMasked() == MotionEvent.ACTION_DOWN) {
                if (!manualMode) sendCommand("M,1");
                sendCommand(String.format(Locale.US, "D,%d,%d", linear, angular));
                v.setPressed(true);
                return true;
            }
            if (event.getActionMasked() == MotionEvent.ACTION_UP ||
                    event.getActionMasked() == MotionEvent.ACTION_CANCEL) {
                sendCommand("D,0,0");
                v.setPressed(false);
                v.performClick();
                return true;
            }
            return true;
        });
        pad.addView(button, padParams());
    }

    private Button padButton(String text, int color) {
        Button button = button(text, color, text.equals("STOP") ? 16 : 30);
        button.setTextColor(text.equals("STOP") ? Color.WHITE : TEXT);
        button.setAllCaps(false);
        button.setPadding(0, 0, 0, 0);
        return button;
    }

    private void addPadSpace(GridLayout pad) {
        pad.addView(new Space(this), padParams());
    }

    private GridLayout.LayoutParams padParams() {
        GridLayout.LayoutParams lp = new GridLayout.LayoutParams();
        lp.width = 0;
        lp.height = 0;
        lp.columnSpec = GridLayout.spec(GridLayout.UNDEFINED, 1f);
        lp.rowSpec = GridLayout.spec(GridLayout.UNDEFINED, 1f);
        lp.setMargins(dp(5), dp(5), dp(5), dp(5));
        return lp;
    }

    private void ensurePermissionsAndScan() {
        if (adapter == null) {
            setConnection("本手机不支持蓝牙", false);
            return;
        }
        if (!hasBlePermissions()) {
            if (Build.VERSION.SDK_INT >= 31) {
                requestPermissions(new String[]{
                        Manifest.permission.BLUETOOTH_SCAN,
                        Manifest.permission.BLUETOOTH_CONNECT
                }, PERMISSION_REQUEST);
            } else {
                requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, PERMISSION_REQUEST);
            }
            return;
        }
        startScan();
    }

    private boolean hasBlePermissions() {
        if (Build.VERSION.SDK_INT >= 31) {
            return checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED &&
                    checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED;
        }
        return checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == PERMISSION_REQUEST && hasBlePermissions()) {
            startScan();
        } else if (requestCode == PERMISSION_REQUEST) {
            setConnection("需要蓝牙权限才能连接", false);
        }
    }

    @SuppressLint("MissingPermission")
    private void startScan() {
        if (!adapter.isEnabled()) {
            startActivity(new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE));
            setConnection("请打开蓝牙后点连接", false);
            return;
        }
        disconnectGatt();
        scanner = adapter.getBluetoothLeScanner();
        if (scanner == null) {
            setConnection("蓝牙扫描不可用", false);
            return;
        }
        scanning = true;
        setConnection("正在搜索 SmartSuitcase…", false);
        scanner.startScan(scanCallback);
        handler.removeCallbacks(scanTimeout);
        handler.postDelayed(scanTimeout, 10000);
    }

    @SuppressLint("MissingPermission")
    private void stopScan() {
        handler.removeCallbacks(scanTimeout);
        if (scanning && scanner != null) scanner.stopScan(scanCallback);
        scanning = false;
    }

    private final ScanCallback scanCallback = new ScanCallback() {
        @SuppressLint("MissingPermission")
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            String name = result.getScanRecord() == null ? null : result.getScanRecord().getDeviceName();
            if (name == null) name = result.getDevice().getName();
            if (DEVICE_NAME.equals(name)) {
                stopScan();
                pendingDevice = result.getDevice();
                setConnection(pendingDevice.getBondState() == BluetoothDevice.BOND_BONDED
                        ? "已配对，正在连接…" : "正在连接，随后输入配对码 123456", false);
                connectDevice(pendingDevice);
            }
        }

        @Override
        public void onScanFailed(int errorCode) {
            scanning = false;
            setConnection("扫描失败：" + errorCode, false);
        }
    };

    @SuppressLint("MissingPermission")
    private void connectDevice(BluetoothDevice device) {
        if (gatt != null) return;
        gatt = device.connectGatt(MainActivity.this, false, gattCallback,
                BluetoothDevice.TRANSPORT_LE);
    }

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        @Override
        public void onConnectionStateChange(BluetoothGatt bluetoothGatt, int status, int newState) {
            if (newState == BluetoothProfile.STATE_CONNECTED && status == BluetoothGatt.GATT_SUCCESS) {
                if (bluetoothGatt.getDevice().getBondState() == BluetoothDevice.BOND_BONDED) {
                    runOnUiThread(() -> setConnection("已加密连接，正在初始化…", false));
                    beginGattSetup(bluetoothGatt);
                } else {
                    runOnUiThread(() -> setConnection("请输入蓝牙配对码 123456", false));
                    bluetoothGatt.getDevice().createBond();
                }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                runOnUiThread(() -> {
                    ready = false;
                    setupStarted = false;
                    handler.removeCallbacks(heartbeat);
                    setConnection("蓝牙已断开，点此重连", false);
                });
                bluetoothGatt.close();
                if (gatt == bluetoothGatt) gatt = null;
            }
        }

        @SuppressLint("MissingPermission")
        @Override
        public void onMtuChanged(BluetoothGatt bluetoothGatt, int mtu, int status) {
            bluetoothGatt.discoverServices();
        }

        @SuppressLint("MissingPermission")
        @Override
        public void onServicesDiscovered(BluetoothGatt bluetoothGatt, int status) {
            BluetoothGattService service = bluetoothGatt.getService(SERVICE_UUID);
            if (status != BluetoothGatt.GATT_SUCCESS || service == null) {
                runOnUiThread(() -> setConnection("设备服务不匹配", false));
                return;
            }
            commandCharacteristic = service.getCharacteristic(COMMAND_UUID);
            BluetoothGattCharacteristic telemetry = service.getCharacteristic(TELEMETRY_UUID);
            if (commandCharacteristic == null || telemetry == null) {
                runOnUiThread(() -> setConnection("控制通道不完整", false));
                return;
            }
            bluetoothGatt.setCharacteristicNotification(telemetry, true);
            BluetoothGattDescriptor descriptor = telemetry.getDescriptor(CCCD_UUID);
            if (descriptor == null) {
                runOnUiThread(() -> setConnection("实时数据通道不可用", false));
                return;
            }
            descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
            bluetoothGatt.writeDescriptor(descriptor);
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt bluetoothGatt, BluetoothGattDescriptor descriptor, int status) {
            if (CCCD_UUID.equals(descriptor.getUuid()) && status == BluetoothGatt.GATT_SUCCESS) {
                runOnUiThread(() -> {
                    ready = true;
                    setConnection("已连接 SmartSuitcase", true);
                    handler.removeCallbacks(heartbeat);
                    handler.post(heartbeat);
                    sendCommand("H");
                });
            }
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt bluetoothGatt,
                                            BluetoothGattCharacteristic characteristic) {
            if (TELEMETRY_UUID.equals(characteristic.getUuid())) {
                String payload = new String(characteristic.getValue(), StandardCharsets.UTF_8);
                runOnUiThread(() -> applyTelemetry(payload));
            }
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt bluetoothGatt,
                                          BluetoothGattCharacteristic characteristic, int status) {
            synchronized (writeQueue) {
                writeBusy = false;
            }
            runOnUiThread(MainActivity.this::drainWrites);
        }
    };

    @SuppressLint("MissingPermission")
    private synchronized void beginGattSetup(BluetoothGatt bluetoothGatt) {
        if (setupStarted || bluetoothGatt != gatt) return;
        setupStarted = true;
        if (!bluetoothGatt.requestMtu(247)) bluetoothGatt.discoverServices();
    }

    private void sendCommand(String command) {
        if (!ready || commandCharacteristic == null || gatt == null) {
            if (!"H".equals(command)) setConnection("尚未连接，无法发送指令", false);
            return;
        }
        synchronized (writeQueue) {
            if (writeQueue.size() > 20) writeQueue.poll();
            writeQueue.offer(command.getBytes(StandardCharsets.UTF_8));
        }
        drainWrites();
    }

    @SuppressLint("MissingPermission")
    private void drainWrites() {
        byte[] data;
        synchronized (writeQueue) {
            if (writeBusy || writeQueue.isEmpty() || !ready) return;
            data = writeQueue.poll();
            writeBusy = true;
        }
        commandCharacteristic.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
        commandCharacteristic.setValue(data);
        if (!gatt.writeCharacteristic(commandCharacteristic)) {
            synchronized (writeQueue) {
                writeBusy = false;
            }
            handler.postDelayed(this::drainWrites, 40);
        }
    }

    private void applyTelemetry(String payload) {
        String[] p = payload.trim().split(",", -1);
        if (p.length < 21 || !"T".equals(p[0])) return;
        try {
            manualMode = integer(p[1]) == 1;
            estop = integer(p[2]) == 1;
            int follow = integer(p[4]);
            int turn = integer(p[5]);
            int remote = integer(p[6]);
            String state = p[7];
            int flags = integer(p[8]);
            float targetM = decimal(p[9]);
            float bearingDeg = (float) Math.toDegrees(decimal(p[10]));
            float frontM = decimal(p[11]);
            float leftM = decimal(p[12]);
            float rightM = decimal(p[13]);
            float batteryPct = decimal(p[14]);
            float batteryV = decimal(p[15]);
            float weightKg = decimal(p[16]);
            float measuredV = decimal(p[17]);
            float measuredW = decimal(p[18]);
            int leftUs = integer(p[19]);
            int rightUs = integer(p[20]);

            stateValue.setText(estop ? "急停锁定" : state);
            stateValue.setTextColor(estop ? Color.rgb(255, 126, 126) : TEXT);
            batteryValue.setText(String.format(Locale.CHINA, "%.0f%%", batteryPct));
            batterySub.setText(String.format(Locale.CHINA, "%.2f V", batteryV));
            weightValue.setText(String.format(Locale.CHINA, "%.1f kg", weightKg));
            targetDistanceValue.setText(validDistance(targetM) ?
                    String.format(Locale.CHINA, "%.2f m", targetM) : "-- m");
            boolean distanceTooFar = (flags & 1) != 0 && validDistance(targetM) && targetM > 5.0f;
            distanceAlert.setVisibility(distanceTooFar ? View.VISIBLE : View.GONE);
            if (distanceTooFar) {
                distanceAlert.setText(String.format(Locale.CHINA,
                        "距离警报  目标已超过 5 m（%.2f m）", targetM));
            }
            targetBearingValue.setText(String.format(Locale.CHINA, "%+.1f°", bearingDeg));
            frontValue.setText(validDistance(frontM) ?
                    String.format(Locale.CHINA, "%.2f m", frontM) : "-- m");
            sideValue.setText(String.format(Locale.CHINA, "左 %.2f m  ·  右 %.2f m", leftM, rightM));
            motionValue.setText(String.format(Locale.CHINA,
                    "速度 %.2f m/s  ·  转速 %.2f rad/s  ·  PWM %d / %d μs",
                    measuredV, measuredW, leftUs, rightUs));

            setModeStyle();
            updateSlider(followSpeed, followSpeedLabel, follow, 0);
            updateSlider(followTurn, followTurnLabel, turn, 1);
            updateSlider(remoteSpeed, remoteSpeedLabel, remote, 2);
            setBadge(uwbBadge, (flags & 1) != 0);
            setBadge(lidarBadge, (flags & 2) != 0);
            setBadge(ultraLeftBadge, (flags & 4) != 0);
            setBadge(ultraRightBadge, (flags & 8) != 0);
            setBadge(fsrBadge, (flags & 16) != 0);
            setBadge(encoderBadge, (flags & 64) != 0);
        } catch (RuntimeException ignored) {
            setConnection("收到一帧无效数据", true);
        }
    }

    private void updateSlider(SeekBar bar, TextView label, int value, int index) {
        if (!sliderTouching[index]) {
            int bounded = Math.max(0, Math.min(100, value));
            bar.setProgress(bounded);
            label.setText(String.format(Locale.CHINA, "%d%%", bounded));
        }
    }

    private void setModeStyle() {
        followButton.setBackground(round(manualMode ? CONTROL : BLUE_DARK, Color.TRANSPARENT, 0));
        manualButton.setBackground(round(manualMode ? BLUE_DARK : CONTROL, Color.TRANSPARENT, 0));
        followButton.setTextColor(manualMode ? TEXT : Color.WHITE);
        manualButton.setTextColor(manualMode ? Color.WHITE : TEXT);
    }

    private void setBadge(TextView badge, boolean ok) {
        badge.setBackground(round(ok ? SENSOR_GREEN : SENSOR_RED, Color.TRANSPARENT, 0));
        badge.setTextColor(ok ? Color.rgb(0, 104, 68) : Color.rgb(158, 25, 31));
    }

    private boolean validDistance(float value) {
        return Float.isFinite(value) && value > 0.01f && value < 50.0f;
    }

    private int integer(String value) {
        return Integer.parseInt(value.trim());
    }

    private float decimal(String value) {
        return Float.parseFloat(value.trim());
    }

    private void setConnection(String message, boolean connected) {
        connectionText.setText(String.format(Locale.CHINA, "%s  %s",
                connected ? "●" : "○", message));
        connectionText.setTextColor(connected ? Color.rgb(90, 220, 157) : MUTED);
        connectButton.setText(connected ? "断开" : "连接");
        connectButton.setBackground(round(connected ? Color.rgb(59, 67, 78) : BLUE, Color.TRANSPARENT, 0));
    }

    @SuppressLint("MissingPermission")
    private void disconnectGatt() {
        stopScan();
        ready = false;
        setupStarted = false;
        handler.removeCallbacks(heartbeat);
        synchronized (writeQueue) {
            writeQueue.clear();
            writeBusy = false;
        }
        if (gatt != null) {
            gatt.disconnect();
            gatt.close();
            gatt = null;
        }
        commandCharacteristic = null;
    }

    @Override
    protected void onDestroy() {
        disconnectGatt();
        handler.removeCallbacksAndMessages(null);
        try {
            unregisterReceiver(bondReceiver);
        } catch (IllegalArgumentException ignored) {
        }
        super.onDestroy();
    }

    private LinearLayout vertical() {
        LinearLayout view = new LinearLayout(this);
        view.setOrientation(LinearLayout.VERTICAL);
        return view;
    }

    private String readAsset(String path) throws IOException {
        try (InputStream input = getAssets().open(path);
             ByteArrayOutputStream output = new ByteArrayOutputStream()) {
            byte[] buffer = new byte[8192];
            int count;
            while ((count = input.read(buffer)) >= 0) {
                output.write(buffer, 0, count);
            }
            return output.toString(StandardCharsets.UTF_8.name());
        }
    }

    private LinearLayout horizontal() {
        LinearLayout view = new LinearLayout(this);
        view.setOrientation(LinearLayout.HORIZONTAL);
        return view;
    }

    private LinearLayout panel() {
        LinearLayout panel = vertical();
        panel.setPadding(dp(14), dp(14), dp(14), dp(14));
        panel.setBackground(round(PANEL, BORDER, 1));
        return panel;
    }

    private TextView text(String value, int sp, int color, boolean bold) {
        TextView view = new TextView(this);
        view.setText(value);
        view.setTextSize(sp);
        view.setTextColor(color);
        view.setGravity(Gravity.CENTER_VERTICAL);
        view.setIncludeFontPadding(false);
        if (bold) view.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        return view;
    }

    private Button button(String value, int color, int sp) {
        Button button = new Button(this);
        button.setText(value);
        button.setTextSize(sp);
        button.setTextColor(Color.WHITE);
        button.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        button.setAllCaps(false);
        button.setBackground(round(color, Color.TRANSPARENT, 0));
        button.setStateListAnimator(null);
        return button;
    }

    private TextView sensorBadge(String value) {
        TextView badge = text(value, 13, Color.rgb(158, 25, 31), true);
        badge.setGravity(Gravity.CENTER);
        badge.setBackground(round(SENSOR_RED, Color.TRANSPARENT, 0));
        return badge;
    }

    private Metric metric(String title, String value, String sub) {
        LinearLayout card = vertical();
        card.setPadding(dp(14), dp(13), dp(14), dp(12));
        card.setBackground(round(PANEL, BORDER, 1));
        TextView titleView = text(title, 14, MUTED, false);
        TextView valueView = text(value, 29, Color.rgb(123, 190, 255), true);
        TextView subView = text(sub, 13, MUTED, false);
        card.addView(titleView, matchHeight(dp(25)));
        card.addView(valueView, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1));
        card.addView(subView, matchHeight(dp(22)));
        return new Metric(card, valueView, subView);
    }

    private void addHalf(LinearLayout row, View view, boolean marginRight) {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(0,
                ViewGroup.LayoutParams.MATCH_PARENT, 1);
        if (marginRight) lp.setMargins(0, 0, dp(5), 0);
        else lp.setMargins(dp(5), 0, 0, 0);
        row.addView(view, lp);
    }

    private void addThird(LinearLayout row, View view, boolean marginRight) {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(0,
                ViewGroup.LayoutParams.MATCH_PARENT, 1);
        if (marginRight) lp.setMargins(0, 0, dp(6), 0);
        row.addView(view, lp);
    }

    private GradientDrawable round(int fill, int stroke, int strokeWidthDp) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(fill);
        drawable.setCornerRadius(dp(7));
        if (strokeWidthDp > 0) drawable.setStroke(dp(strokeWidthDp), stroke);
        return drawable;
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private LinearLayout.LayoutParams matchWrap() {
        return new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
    }

    private LinearLayout.LayoutParams matchHeight(int height) {
        return new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, height);
    }

    private LinearLayout.LayoutParams withMargins(LinearLayout.LayoutParams lp,
                                                  int left, int top, int right, int bottom) {
        lp.setMargins(dp(left), dp(top), dp(right), dp(bottom));
        return lp;
    }

    private static final class Metric {
        final LinearLayout view;
        final TextView value;
        final TextView sub;

        Metric(LinearLayout view, TextView value, TextView sub) {
            this.view = view;
            this.value = value;
            this.sub = sub;
        }
    }
}
