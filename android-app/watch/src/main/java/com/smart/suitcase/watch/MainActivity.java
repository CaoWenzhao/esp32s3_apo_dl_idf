package com.smart.suitcase.watch;

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
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.graphics.Rect;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.StateListDrawable;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.provider.Settings;
import android.view.Gravity;
import android.view.HapticFeedbackConstants;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowInsets;
import android.view.WindowInsetsController;
import android.view.WindowManager;
import android.window.OnBackInvokedCallback;
import android.window.OnBackInvokedDispatcher;
import android.view.animation.Animation;
import android.view.animation.AnimationSet;
import android.view.animation.DecelerateInterpolator;
import android.view.animation.TranslateAnimation;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.GridLayout;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.Space;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ViewFlipper;

import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Locale;
import java.util.Queue;
import java.util.UUID;

public class MainActivity extends Activity {
    private static final String DEVICE_NAME = "SmartSuitcase";
    private static final UUID SERVICE_UUID =
            UUID.fromString("5f6d1000-8f3e-4c21-a7b2-3d4e5f607182");
    private static final UUID COMMAND_UUID =
            UUID.fromString("5f6d1001-8f3e-4c21-a7b2-3d4e5f607182");
    private static final UUID TELEMETRY_UUID =
            UUID.fromString("5f6d1002-8f3e-4c21-a7b2-3d4e5f607182");
    private static final UUID CCCD_UUID =
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
    private static final int PERMISSION_REQUEST = 51;

    private static final int BG = Color.rgb(6, 8, 11);
    private static final int PANEL = Color.rgb(23, 27, 33);
    private static final int PANEL_ACTIVE = Color.rgb(26, 77, 116);
    private static final int BORDER = Color.rgb(58, 66, 77);
    private static final int TEXT = Color.rgb(242, 245, 248);
    private static final int MUTED = Color.rgb(166, 176, 188);
    private static final int BLUE = Color.rgb(34, 139, 230);
    private static final int GREEN = Color.rgb(0, 151, 102);
    private static final int RED = Color.rgb(204, 35, 42);
    private static final int AMBER = Color.rgb(237, 165, 46);

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final Queue<byte[]> writeQueue = new ArrayDeque<>();

    private BluetoothAdapter adapter;
    private BluetoothLeScanner scanner;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic commandCharacteristic;
    private boolean scanning;
    private boolean ready;
    private boolean setupStarted;
    private boolean writeBusy;
    private boolean manualDisconnect;
    private boolean manualMode;
    private boolean estop = true;
    private OnBackInvokedCallback backCallback;

    private ViewFlipper pages;
    private View leftEdgeGlow;
    private View rightEdgeGlow;
    private TextView connectionDot;
    private TextView connectionText;
    private Button connectButton;
    private TextView operatingMode;
    private FrameLayout batteryBox;
    private View batteryFill;
    private TextView batteryValue;
    private TextView weightValue;
    private TextView alertValue;
    private TextView summaryDistance;
    private TextView summaryBearing;
    private TextView targetDistance;
    private TextView targetBearing;
    private TextView frontDistance;
    private TextView leftDistance;
    private TextView rightDistance;
    private TextView motionValue;
    private TextView pwmValue;
    private TextView modeDetail;
    private TextView uwbBadge;
    private TextView lidarBadge;
    private TextView ultraLeftBadge;
    private TextView ultraRightBadge;
    private TextView fsrBadge;
    private TextView encoderBadge;
    private Button followButton;
    private Button manualButton;
    private SeekBar followSpeed;
    private SeekBar followTurn;
    private SeekBar remoteSpeed;
    private TextView followSpeedLabel;
    private TextView followTurnLabel;
    private TextView remoteSpeedLabel;
    private final boolean[] sliderTouching = new boolean[3];

    private int currentPage;
    private int pageSidePadding;
    private long lastRotaryAt;
    private float touchDownX;
    private float touchDownY;
    private long touchDownAt;

    private final Runnable scanTimeout = new Runnable() {
        @Override
        public void run() {
            stopScan();
            setConnection("未找到，重试中", false);
            if (!manualDisconnect) handler.postDelayed(MainActivity.this::ensurePermissionsAndScan, 1200);
        }
    };

    private final Runnable heartbeat = new Runnable() {
        @Override
        public void run() {
            if (ready) {
                if (writeQueue.size() < 6) sendCommand("H");
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
        if (Build.VERSION.SDK_INT >= 29) {
            getWindow().setNavigationBarContrastEnforced(false);
        }
        if (Build.VERSION.SDK_INT >= 30) {
            getWindow().setDecorFitsSystemWindows(false);
        }
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        buildUi();
        getWindow().getDecorView().post(this::hideSystemBars);
        BluetoothManager manager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        adapter = manager == null ? null : manager.getAdapter();
        ensurePermissionsAndScan();
    }

    private void buildUi() {
        LinearLayout root = vertical();
        pageSidePadding = getResources().getConfiguration().isScreenRound() ? dp(22) : dp(14);
        root.setPadding(0, dp(6), 0, dp(5));
        root.setBackgroundColor(BG);
        root.setFocusable(true);
        root.setFocusableInTouchMode(true);
        root.requestFocus();
        root.setOnApplyWindowInsetsListener((view, insets) -> {
            int bottomInset;
            if (Build.VERSION.SDK_INT >= 30) {
                bottomInset = insets.getInsets(WindowInsets.Type.navigationBars()).bottom;
            } else {
                bottomInset = insets.getSystemWindowInsetBottom();
            }
            view.setPadding(0, dp(6), 0, dp(5) + bottomInset);
            return insets;
        });

        pages = new ViewFlipper(this);
        pages.setMeasureAllChildren(false);
        pages.addView(buildOverviewPage());
        pages.addView(buildMotionPage());
        pages.addView(buildFollowPage());
        pages.addView(buildRemotePage());
        pages.addView(buildSensorPage());
        root.addView(pages, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1));

        FrameLayout shell = new FrameLayout(this);
        shell.setBackgroundColor(BG);
        shell.addView(root, new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT));
        leftEdgeGlow = buildEdgeGlow(true);
        rightEdgeGlow = buildEdgeGlow(false);
        FrameLayout.LayoutParams leftGlowLp = new FrameLayout.LayoutParams(
                dp(24), ViewGroup.LayoutParams.MATCH_PARENT, Gravity.START);
        FrameLayout.LayoutParams rightGlowLp = new FrameLayout.LayoutParams(
                dp(24), ViewGroup.LayoutParams.MATCH_PARENT, Gravity.END);
        shell.addView(leftEdgeGlow, leftGlowLp);
        shell.addView(rightEdgeGlow, rightGlowLp);
        setContentView(shell);
        claimHorizontalGestures(shell);
        showPage(0, false);
    }

    private void claimHorizontalGestures(View root) {
        if (Build.VERSION.SDK_INT >= 29) {
            root.addOnLayoutChangeListener((view, left, top, right, bottom,
                                            oldLeft, oldTop, oldRight, oldBottom) -> {
                int width = view.getWidth();
                int height = view.getHeight();
                int edge = Math.min(dp(48), width / 2);
                view.setSystemGestureExclusionRects(Arrays.asList(
                        new Rect(0, 0, edge, height),
                        new Rect(width - edge, 0, width, height)));
            });
        }
        if (Build.VERSION.SDK_INT >= 33) {
            backCallback = () -> {
                // Horizontal gestures are reserved for in-app paging.
            };
            getOnBackInvokedDispatcher().registerOnBackInvokedCallback(
                    OnBackInvokedDispatcher.PRIORITY_DEFAULT, backCallback);
        }
    }

    private void hideSystemBars() {
        if (Build.VERSION.SDK_INT >= 30) {
            try {
                View decor = getWindow().getDecorView();
                WindowInsetsController controller = decor == null
                        ? null : decor.getWindowInsetsController();
                if (controller != null) {
                    controller.hide(WindowInsets.Type.statusBars() |
                            WindowInsets.Type.navigationBars());
                    controller.setSystemBarsBehavior(
                            WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
                    return;
                }
            } catch (RuntimeException ignored) {
                // Some customized Android 11 builds expose the API before their decor is ready.
            }
        }
        getWindow().getDecorView().setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_FULLSCREEN |
                        View.SYSTEM_UI_FLAG_HIDE_NAVIGATION |
                        View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY |
                        View.SYSTEM_UI_FLAG_LAYOUT_STABLE |
                        View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN |
                        View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION);
    }

    @Override
    protected void onResume() {
        super.onResume();
        hideSystemBars();
        handler.postDelayed(this::hideSystemBars, 250);
        handler.postDelayed(() -> {
            if (adapter != null && hasBlePermissions() && adapter.isEnabled() &&
                    !manualDisconnect && !ready && gatt == null && !scanning) {
                startScan();
            }
        }, 400);
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) hideSystemBars();
    }

    private View buildOverviewPage() {
        LinearLayout page = compactPage();
        addPageTitle(page, "总览");
        LinearLayout identity = horizontal();
        identity.setGravity(Gravity.CENTER_VERTICAL);
        connectionDot = text("●", 8, RED, true);
        identity.addView(connectionDot, new LinearLayout.LayoutParams(dp(11), dp(24)));
        connectionText = text("未连接", 9, TEXT, true);
        identity.addView(connectionText, new LinearLayout.LayoutParams(0, dp(24), .8f));
        operatingMode = text("急停锁定", 10, RED, true);
        operatingMode.setGravity(Gravity.CENTER);
        identity.addView(operatingMode, new LinearLayout.LayoutParams(0, dp(24), 1.0f));
        connectButton = button("连接", BLUE, 9);
        connectButton.setOnClickListener(v -> {
            if (ready || gatt != null) {
                manualDisconnect = true;
                disconnectGatt();
            } else {
                manualDisconnect = false;
                ensurePermissionsAndScan();
            }
        });
        identity.addView(connectButton, new LinearLayout.LayoutParams(dp(43), dp(24)));
        page.addView(identity, matchHeight(dp(25)));

        LinearLayout emergency = horizontal();
        Button stop = button("紧急停止", RED, 10);
        stop.setOnClickListener(v -> sendCommand("E"));
        emergency.addView(stop, new LinearLayout.LayoutParams(0, dp(31), 1));
        Button arm = button("解除急停", GREEN, 10);
        arm.setOnClickListener(v -> sendCommand("A"));
        LinearLayout.LayoutParams armLp = new LinearLayout.LayoutParams(0, dp(31), 1);
        armLp.setMargins(dp(5), 0, 0, 0);
        emergency.addView(arm, armLp);
        page.addView(emergency, withMargins(matchWrap(), 0, 2, 0, 3));

        alertValue = text("目标距离超过 5 m", 10, Color.WHITE, true);
        alertValue.setGravity(Gravity.CENTER);
        alertValue.setBackground(round(RED, RED, 20));
        alertValue.setVisibility(View.GONE);
        page.addView(alertValue, withMargins(matchHeight(dp(20)), 0, 0, 0, dp(2)));

        LinearLayout primary = horizontal();
        batteryValue = batteryMetric(primary, "电量", "100%", true);
        weightValue = metric(primary, "重量", "-- kg", false);
        page.addView(primary, withMargins(matchHeight(dp(36)), 0, 0, 0, dp(3)));
        setBatteryProgress(100);

        LinearLayout target = horizontal();
        summaryDistance = metric(target, "距离", "-- m", true);
        summaryBearing = metric(target, "方向", "--°", false);
        page.addView(target, matchHeight(dp(36)));
        return page;
    }

    private View buildFollowPage() {
        LinearLayout page = compactPage();
        addPageTitle(page, "跟随");

        LinearLayout modes = horizontal();
        followButton = button("跟随", PANEL_ACTIVE, 11);
        manualButton = button("遥控", PANEL, 11);
        followButton.setOnClickListener(v -> {
            manualMode = false;
            setModeStyle();
            sendCommand("M,0");
        });
        manualButton.setOnClickListener(v -> {
            manualMode = true;
            setModeStyle();
            sendCommand("M,1");
        });
        modes.addView(followButton, new LinearLayout.LayoutParams(0, dp(31), 1));
        LinearLayout.LayoutParams manualLp = new LinearLayout.LayoutParams(0, dp(31), 1);
        manualLp.setMargins(dp(5), 0, 0, 0);
        modes.addView(manualButton, manualLp);
        page.addView(modes, withMargins(matchWrap(), 0, 5, 0, 6));

        LinearLayout target = horizontal();
        targetDistance = metric(target, "目标距离", "-- m", true);
        targetBearing = metric(target, "目标方向", "--°", false);
        page.addView(target, withMargins(matchHeight(dp(39)), 0, 0, 0, 6));

        LinearLayout obstacle = horizontal();
        frontDistance = metric(obstacle, "前向雷达", "-- m", true);
        LinearLayout sides = vertical();
        sides.setPadding(dp(5), dp(2), dp(5), dp(2));
        sides.setBackground(round(PANEL, BORDER, 18));
        leftDistance = text("左 -- m", 10, TEXT, true);
        rightDistance = text("右 -- m", 10, TEXT, true);
        leftDistance.setGravity(Gravity.CENTER);
        rightDistance.setGravity(Gravity.CENTER);
        sides.addView(leftDistance, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1));
        sides.addView(rightDistance, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1));
        obstacle.addView(sides, halfParams(false));
        page.addView(obstacle, matchHeight(dp(39)));
        return page;
    }

    private View buildRemotePage() {
        LinearLayout page = page();
        addPageTitle(page, "遥控");
        GridLayout pad = new GridLayout(this);
        pad.setColumnCount(3);
        pad.setRowCount(3);
        pad.setAlignmentMode(GridLayout.ALIGN_BOUNDS);
        addPadSpace(pad);
        addDriveButton(pad, "▲", 100, 0);
        addPadSpace(pad);
        addDriveButton(pad, "◀", 0, 100);
        Button stop = padButton("STOP", RED);
        stop.setOnClickListener(v -> sendCommand("D,0,0"));
        pad.addView(stop, padParams());
        addDriveButton(pad, "▶", 0, -100);
        addPadSpace(pad);
        addDriveButton(pad, "▼", -100, 0);
        addPadSpace(pad);
        page.addView(pad, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1));
        return page;
    }

    private View buildSensorPage() {
        LinearLayout page = compactPage();
        addPageTitle(page, "传感器");
        LinearLayout row1 = horizontal();
        uwbBadge = sensorBadge("UWB");
        lidarBadge = sensorBadge("激光雷达");
        addSensor(row1, uwbBadge, true);
        addSensor(row1, lidarBadge, false);
        page.addView(row1, withMargins(matchHeight(dp(38)), 0, 5, 0, 6));

        LinearLayout row2 = horizontal();
        ultraLeftBadge = sensorBadge("左超声波");
        ultraRightBadge = sensorBadge("右超声波");
        addSensor(row2, ultraLeftBadge, true);
        addSensor(row2, ultraRightBadge, false);
        page.addView(row2, withMargins(matchHeight(dp(38)), 0, 0, 0, 6));

        LinearLayout row3 = horizontal();
        fsrBadge = sensorBadge("压力");
        encoderBadge = sensorBadge("编码器");
        addSensor(row3, fsrBadge, true);
        addSensor(row3, encoderBadge, false);
        page.addView(row3, matchHeight(dp(38)));
        return page;
    }

    private View buildMotionPage() {
        LinearLayout page = compactPage();
        addPageTitle(page, "运动");
        modeDetail = text("状态 --", 10, TEXT, true);
        modeDetail.setGravity(Gravity.CENTER);
        modeDetail.setBackground(round(PANEL, BORDER, 18));
        page.addView(modeDetail, withMargins(matchHeight(dp(20)), 0, 0, 0, dp(2)));

        motionValue = text("速度 -- m/s  ·  转速 -- rad/s", 10, TEXT, false);
        motionValue.setGravity(Gravity.CENTER);
        motionValue.setBackground(round(PANEL, BORDER, 18));
        page.addView(motionValue, withMargins(matchHeight(dp(24)), 0, 0, 0, dp(2)));

        pwmValue = text("电机脉宽 -- / -- μs", 10, TEXT, false);
        pwmValue.setGravity(Gravity.CENTER);
        pwmValue.setBackground(round(PANEL, BORDER, 18));
        page.addView(pwmValue, withMargins(matchHeight(dp(20)), 0, 0, 0, dp(1)));

        followSpeed = addSlider(page, "跟随速度", 75, 0);
        followTurn = addSlider(page, "跟随转向", 70, 1);
        remoteSpeed = addSlider(page, "遥控速度", 80, 2);
        return page;
    }

    private SeekBar addSlider(LinearLayout parent, String title, int initial, int index) {
        LinearLayout row = horizontal();
        row.setGravity(Gravity.CENTER_VERTICAL);
        TextView name = text(title, 9, MUTED, false);
        TextView value = text(initial + "%", 9, TEXT, true);
        value.setGravity(Gravity.END | Gravity.CENTER_VERTICAL);
        row.addView(name, new LinearLayout.LayoutParams(dp(46), dp(20)));

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
                    handler.postDelayed(speedSender, 80);
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
        row.addView(seek, new LinearLayout.LayoutParams(0, dp(20), 1));
        row.addView(value, new LinearLayout.LayoutParams(dp(32), dp(20)));
        parent.addView(row, matchHeight(dp(20)));
        if (index == 0) followSpeedLabel = value;
        if (index == 1) followTurnLabel = value;
        if (index == 2) remoteSpeedLabel = value;
        return seek;
    }

    @SuppressLint("ClickableViewAccessibility")
    private void addDriveButton(GridLayout pad, String symbol, int linear, int angular) {
        Button button = padButton(symbol, PANEL);
        button.setOnTouchListener((v, event) -> {
            if (event.getActionMasked() == MotionEvent.ACTION_DOWN) {
                if (!manualMode) {
                    manualMode = true;
                    setModeStyle();
                    sendCommand("M,1");
                }
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

    private void addPadSpace(GridLayout pad) {
        pad.addView(new Space(this), padParams());
    }

    private Button padButton(String label, int color) {
        Button button = button(label, color, "STOP".equals(label) ? 10 : 18);
        button.setPadding(0, 0, 0, 0);
        StateListDrawable background = new StateListDrawable();
        background.addState(new int[]{android.R.attr.state_pressed},
                round(PANEL_ACTIVE, BORDER, 18));
        background.addState(new int[]{}, round(color, color == RED ? RED : BORDER, 18));
        button.setBackground(background);
        return button;
    }

    private GridLayout.LayoutParams padParams() {
        GridLayout.LayoutParams lp = new GridLayout.LayoutParams();
        lp.width = 0;
        lp.height = 0;
        lp.columnSpec = GridLayout.spec(GridLayout.UNDEFINED, 1f);
        lp.rowSpec = GridLayout.spec(GridLayout.UNDEFINED, 1f);
        lp.setMargins(dp(3), dp(3), dp(3), dp(3));
        return lp;
    }

    private void showPage(int index) {
        showPage(index, true);
    }

    private void showPage(int index, boolean animate) {
        if (pages == null) return;
        int count = pages.getChildCount();
        if (index < 0) {
            playEdgeGlow(true);
            return;
        }
        if (index >= count) {
            playEdgeGlow(false);
            return;
        }
        boolean forward = index > currentPage;
        if (animate && index != currentPage) {
            pages.setInAnimation(pageAnimation(forward, true));
            pages.setOutAnimation(pageAnimation(forward, false));
        } else {
            pages.setInAnimation(null);
            pages.setOutAnimation(null);
        }
        currentPage = index;
        pages.setDisplayedChild(currentPage);
    }

    private Animation pageAnimation(boolean forward, boolean incoming) {
        AnimationSet set = new AnimationSet(true);
        float fromX;
        float toX;
        if (incoming) {
            fromX = forward ? 1f : -1f;
            toX = 0f;
        } else {
            fromX = 0f;
            toX = forward ? -1f : 1f;
        }
        TranslateAnimation slide = new TranslateAnimation(
                Animation.RELATIVE_TO_SELF, fromX,
                Animation.RELATIVE_TO_SELF, toX,
                Animation.RELATIVE_TO_SELF, 0f,
                Animation.RELATIVE_TO_SELF, 0f);
        set.addAnimation(slide);
        set.setDuration(240);
        set.setInterpolator(new DecelerateInterpolator());
        return set;
    }

    private View buildEdgeGlow(boolean left) {
        View glow = new View(this);
        GradientDrawable.Orientation orientation = left
                ? GradientDrawable.Orientation.LEFT_RIGHT
                : GradientDrawable.Orientation.RIGHT_LEFT;
        glow.setBackground(new GradientDrawable(orientation,
                new int[]{Color.argb(210, 38, 150, 255),
                        Color.argb(90, 38, 150, 255),
                        Color.TRANSPARENT}));
        glow.setAlpha(0f);
        glow.setVisibility(View.GONE);
        glow.setEnabled(false);
        glow.setImportantForAccessibility(View.IMPORTANT_FOR_ACCESSIBILITY_NO);
        return glow;
    }

    private void playEdgeGlow(boolean left) {
        View glow = left ? leftEdgeGlow : rightEdgeGlow;
        if (glow == null) return;
        glow.animate().cancel();
        glow.setVisibility(View.VISIBLE);
        glow.setPivotX(left ? 0f : glow.getWidth());
        glow.setScaleX(.45f);
        glow.setAlpha(0f);
        glow.animate()
                .alpha(1f)
                .scaleX(1f)
                .setDuration(90)
                .withEndAction(() -> glow.animate()
                        .alpha(0f)
                        .scaleX(1.15f)
                        .setDuration(280)
                        .withEndAction(() -> glow.setVisibility(View.GONE))
                        .start())
                .start();
        pages.performHapticFeedback(HapticFeedbackConstants.LONG_PRESS);
    }

    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_SCROLL) {
            float axis = event.getAxisValue(MotionEvent.AXIS_SCROLL);
            boolean rotary = event.isFromSource(InputDevice.SOURCE_ROTARY_ENCODER);
            if (rotary || axis != 0f) {
                long now = System.currentTimeMillis();
                if (Math.abs(axis) > 0.01f && now - lastRotaryAt > 240) {
                    showPage(currentPage + (axis < 0 ? 1 : -1));
                    lastRotaryAt = now;
                }
                return true;
            }
        }
        return super.dispatchGenericMotionEvent(event);
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (keyCode == KeyEvent.KEYCODE_NAVIGATE_NEXT ||
                keyCode == KeyEvent.KEYCODE_DPAD_RIGHT ||
                keyCode == KeyEvent.KEYCODE_PAGE_DOWN) {
            showPage(currentPage + 1);
            return true;
        }
        if (keyCode == KeyEvent.KEYCODE_NAVIGATE_PREVIOUS ||
                keyCode == KeyEvent.KEYCODE_DPAD_LEFT ||
                keyCode == KeyEvent.KEYCODE_PAGE_UP) {
            showPage(currentPage - 1);
            return true;
        }
        return super.onKeyDown(keyCode, event);
    }

    @SuppressWarnings("deprecation")
    @Override
    public void onBackPressed() {
        // Keep the watch app open; horizontal swipes only change pages.
    }

    @Override
    public boolean dispatchTouchEvent(MotionEvent event) {
        if (event.getActionMasked() == MotionEvent.ACTION_DOWN) {
            touchDownX = event.getX();
            touchDownY = event.getY();
            touchDownAt = System.currentTimeMillis();
        }
        boolean handled = super.dispatchTouchEvent(event);
        if (event.getActionMasked() == MotionEvent.ACTION_UP) {
            float dx = event.getX() - touchDownX;
            float dy = event.getY() - touchDownY;
            long elapsed = System.currentTimeMillis() - touchDownAt;
            if (elapsed < 900 && Math.abs(dx) > dp(62) && Math.abs(dx) > Math.abs(dy) * 1.35f) {
                showPage(currentPage + (dx < 0 ? 1 : -1));
                return true;
            }
        }
        return handled;
    }

    private void ensurePermissionsAndScan() {
        if (adapter == null) {
            setConnection("不支持蓝牙", false);
            return;
        }
        if (!hasBlePermissions()) {
            if (Build.VERSION.SDK_INT >= 31) {
                requestPermissions(new String[]{
                        Manifest.permission.BLUETOOTH_SCAN,
                        Manifest.permission.BLUETOOTH_CONNECT
                }, PERMISSION_REQUEST);
            } else {
                requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                        PERMISSION_REQUEST);
            }
            return;
        }
        startScan();
    }

    private boolean hasBlePermissions() {
        if (Build.VERSION.SDK_INT >= 31) {
            return checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) ==
                    PackageManager.PERMISSION_GRANTED &&
                    checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) ==
                            PackageManager.PERMISSION_GRANTED;
        }
        return checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) ==
                PackageManager.PERMISSION_GRANTED;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions,
                                           int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == PERMISSION_REQUEST && hasBlePermissions()) {
            startScan();
        } else if (requestCode == PERMISSION_REQUEST) {
            setConnection("需要蓝牙权限", false);
        }
    }

    @SuppressLint("MissingPermission")
    private void startScan() {
        if (!adapter.isEnabled()) {
            Toast.makeText(this, "请先打开蓝牙", Toast.LENGTH_SHORT).show();
            setConnection("请打开蓝牙", false);
            startActivity(new Intent(Settings.ACTION_BLUETOOTH_SETTINGS));
            return;
        }
        manualDisconnect = false;
        closeGatt();
        scanner = adapter.getBluetoothLeScanner();
        if (scanner == null) {
            setConnection("扫描不可用", false);
            return;
        }
        scanning = true;
        setConnection("搜索中", false);
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
            String name = result.getScanRecord() == null
                    ? null : result.getScanRecord().getDeviceName();
            if (name == null) name = result.getDevice().getName();
            if (DEVICE_NAME.equals(name)) {
                stopScan();
                setConnection("正在连接", false);
                connectDevice(result.getDevice());
            }
        }

        @Override
        public void onScanFailed(int errorCode) {
            scanning = false;
            setConnection("扫描失败 " + errorCode, false);
        }
    };

    @SuppressLint("MissingPermission")
    private void connectDevice(BluetoothDevice device) {
        if (gatt != null) return;
        gatt = device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE);
    }

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        @Override
        public void onConnectionStateChange(BluetoothGatt bluetoothGatt, int status,
                                            int newState) {
            if (newState == BluetoothProfile.STATE_CONNECTED &&
                    status == BluetoothGatt.GATT_SUCCESS) {
                runOnUiThread(() -> setConnection("初始化中", false));
                beginGattSetup(bluetoothGatt);
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                bluetoothGatt.close();
                if (gatt == bluetoothGatt) gatt = null;
                runOnUiThread(() -> {
                    ready = false;
                    setupStarted = false;
                    commandCharacteristic = null;
                    handler.removeCallbacks(heartbeat);
                    resetTelemetry();
                    setConnection(manualDisconnect ? "已断开" : "重连中", false);
                    if (!manualDisconnect) {
                        handler.postDelayed(MainActivity.this::ensurePermissionsAndScan, 700);
                    }
                });
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
                runOnUiThread(() -> setConnection("服务不匹配", false));
                return;
            }
            commandCharacteristic = service.getCharacteristic(COMMAND_UUID);
            BluetoothGattCharacteristic telemetry = service.getCharacteristic(TELEMETRY_UUID);
            if (commandCharacteristic == null || telemetry == null) {
                runOnUiThread(() -> setConnection("通道不完整", false));
                return;
            }
            bluetoothGatt.setCharacteristicNotification(telemetry, true);
            BluetoothGattDescriptor descriptor = telemetry.getDescriptor(CCCD_UUID);
            if (descriptor == null) {
                runOnUiThread(() -> setConnection("实时数据不可用", false));
                return;
            }
            descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
            bluetoothGatt.writeDescriptor(descriptor);
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt bluetoothGatt,
                                      BluetoothGattDescriptor descriptor, int status) {
            if (CCCD_UUID.equals(descriptor.getUuid()) &&
                    status == BluetoothGatt.GATT_SUCCESS) {
                runOnUiThread(() -> {
                    ready = true;
                    setConnection("已连接", true);
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
                                          BluetoothGattCharacteristic characteristic,
                                          int status) {
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
            if (!"H".equals(command)) setConnection("尚未连接", false);
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
            int battery = Math.max(0, Math.min(100, Math.round(decimal(p[14]))));
            float weightKg = decimal(p[16]);
            float measuredV = decimal(p[17]);
            float measuredW = decimal(p[18]);
            int leftUs = integer(p[19]);
            int rightUs = integer(p[20]);

            batteryValue.setText(String.format(Locale.CHINA, "%d%%", battery));
            setBatteryProgress(battery);
            weightValue.setText(String.format(Locale.CHINA, "%.1f kg", weightKg));
            String distance = validDistance(targetM)
                    ? String.format(Locale.CHINA, "%.2f m", targetM) : "-- m";
            String bearing = String.format(Locale.CHINA, "%+.1f°", bearingDeg);
            summaryDistance.setText(distance);
            targetDistance.setText(distance);
            summaryBearing.setText(bearing);
            targetBearing.setText(bearing);
            frontDistance.setText(validDistance(frontM)
                    ? String.format(Locale.CHINA, "%.2f m", frontM) : "-- m");
            leftDistance.setText(validDistance(leftM)
                    ? String.format(Locale.CHINA, "左 %.2f m", leftM) : "左 -- m");
            rightDistance.setText(validDistance(rightM)
                    ? String.format(Locale.CHINA, "右 %.2f m", rightM) : "右 -- m");
            motionValue.setText(String.format(Locale.CHINA,
                    "速度 %.2f m/s\n转速 %.2f rad/s", measuredV, measuredW));
            pwmValue.setText(String.format(Locale.CHINA,
                    "电机脉宽 %d / %d μs", leftUs, rightUs));

            setOperatingMode(state);
            modeDetail.setText(String.format(Locale.CHINA, "状态 %s", state));
            boolean distanceTooFar = (flags & 1) != 0 &&
                    validDistance(targetM) && targetM > 5f;
            alertValue.setVisibility(distanceTooFar ? View.VISIBLE : View.GONE);

            updateSlider(followSpeed, followSpeedLabel, follow, 0);
            updateSlider(followTurn, followTurnLabel, turn, 1);
            updateSlider(remoteSpeed, remoteSpeedLabel, remote, 2);
            setModeStyle();
            setBadge(uwbBadge, (flags & 1) != 0);
            setBadge(lidarBadge, (flags & 2) != 0);
            setBadge(ultraLeftBadge, (flags & 4) != 0);
            setBadge(ultraRightBadge, (flags & 8) != 0);
            setBadge(fsrBadge, (flags & 16) != 0);
            setBadge(encoderBadge, (flags & 64) != 0);
        } catch (RuntimeException ignored) {
            setConnection("数据帧异常", true);
        }
    }

    private void setOperatingMode(String state) {
        String upper = state == null ? "" : state.toUpperCase(Locale.ROOT);
        String label;
        int color;
        if (estop || upper.contains("ESTOP")) {
            label = "急停锁定";
            color = RED;
        } else if (upper.contains("AVOID")) {
            label = "避障模式";
            color = AMBER;
        } else if (upper.contains("SEARCH")) {
            label = "搜索模式";
            color = AMBER;
        } else if (manualMode || upper.contains("MANUAL")) {
            label = "遥控模式";
            color = BLUE;
        } else {
            label = "跟随模式";
            color = GREEN;
        }
        operatingMode.setText(label);
        operatingMode.setTextColor(color);
    }

    private void updateSlider(SeekBar bar, TextView label, int value, int index) {
        if (!sliderTouching[index]) {
            int bounded = Math.max(0, Math.min(100, value));
            bar.setProgress(bounded);
            label.setText(String.format(Locale.CHINA, "%d%%", bounded));
        }
    }

    private void setModeStyle() {
        followButton.setBackground(round(manualMode ? PANEL : PANEL_ACTIVE, BORDER, 18));
        manualButton.setBackground(round(manualMode ? PANEL_ACTIVE : PANEL, BORDER, 18));
    }

    private void setBadge(TextView badge, boolean ok) {
        badge.setTextColor(ok ? Color.rgb(120, 239, 180) : Color.rgb(255, 139, 144));
        badge.setBackground(round(
                ok ? Color.rgb(17, 70, 50) : Color.rgb(78, 34, 38),
                ok ? GREEN : RED, 18));
        String base = String.valueOf(badge.getTag());
        badge.setText((ok ? "● " : "● ") + base);
    }

    private void setConnection(String label, boolean connected) {
        connectionText.setText(label);
        connectionDot.setTextColor(connected ? GREEN : RED);
        connectButton.setText(connected ? "断开" : "连接");
        if (!connected && operatingMode != null && !ready) {
            operatingMode.setText(estop ? "急停锁定" : "状态未知");
            operatingMode.setTextColor(estop ? RED : MUTED);
        }
    }

    private void resetTelemetry() {
        manualMode = false;
        estop = true;
        batteryValue.setText("100%");
        setBatteryProgress(100);
        weightValue.setText("-- kg");
        summaryDistance.setText("-- m");
        summaryBearing.setText("--°");
        targetDistance.setText("-- m");
        targetBearing.setText("--°");
        frontDistance.setText("-- m");
        leftDistance.setText("左 -- m");
        rightDistance.setText("右 -- m");
        motionValue.setText("速度 -- m/s  ·  转速 -- rad/s");
        pwmValue.setText("电机脉宽 -- / -- μs");
        modeDetail.setText("状态 --");
        alertValue.setVisibility(View.GONE);
        setBadge(uwbBadge, false);
        setBadge(lidarBadge, false);
        setBadge(ultraLeftBadge, false);
        setBadge(ultraRightBadge, false);
        setBadge(fsrBadge, false);
        setBadge(encoderBadge, false);
        setModeStyle();
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
        } else {
            closeGatt();
            resetTelemetry();
            setConnection("已断开", false);
        }
    }

    @SuppressLint("MissingPermission")
    private void closeGatt() {
        if (gatt != null) {
            gatt.close();
            gatt = null;
        }
        commandCharacteristic = null;
        setupStarted = false;
        ready = false;
        synchronized (writeQueue) {
            writeQueue.clear();
            writeBusy = false;
        }
    }

    @Override
    protected void onDestroy() {
        manualDisconnect = true;
        stopScan();
        handler.removeCallbacksAndMessages(null);
        closeGatt();
        if (Build.VERSION.SDK_INT >= 33 && backCallback != null) {
            getOnBackInvokedDispatcher().unregisterOnBackInvokedCallback(backCallback);
            backCallback = null;
        }
        super.onDestroy();
    }

    private LinearLayout page() {
        LinearLayout page = vertical();
        page.setGravity(Gravity.CENTER);
        page.setPadding(pageSidePadding, dp(2), pageSidePadding, dp(2));
        return page;
    }

    private LinearLayout compactPage() {
        LinearLayout page = page();
        page.setGravity(Gravity.TOP | Gravity.CENTER_HORIZONTAL);
        return page;
    }

    private void addPageTitle(LinearLayout page, String title) {
        TextView titleView = text(title, 13, TEXT, true);
        titleView.setGravity(Gravity.CENTER);
        page.addView(titleView, withMargins(matchHeight(dp(20)), 0, 0, 0, dp(2)));
    }

    private TextView metric(LinearLayout row, String label, String value, boolean first) {
        LinearLayout box = vertical();
        box.setGravity(Gravity.CENTER);
        box.setBackground(round(PANEL, BORDER, 18));
        TextView name = text(label, 8, MUTED, false);
        name.setGravity(Gravity.CENTER);
        TextView data = text(value, 13, TEXT, true);
        data.setGravity(Gravity.CENTER);
        box.addView(name, matchHeight(dp(12)));
        box.addView(data, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1));
        row.addView(box, halfParams(first));
        return data;
    }

    private TextView batteryMetric(LinearLayout row, String label,
                                   String value, boolean first) {
        batteryBox = new FrameLayout(this);
        batteryBox.setBackground(round(PANEL, BORDER, 18));
        batteryBox.setClipToOutline(true);

        batteryFill = new View(this);
        batteryFill.setBackground(round(Color.rgb(24, 85, 143),
                Color.rgb(24, 85, 143), 18));
        batteryBox.addView(batteryFill, new FrameLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT));

        LinearLayout content = vertical();
        content.setGravity(Gravity.CENTER);
        TextView name = text(label, 8, MUTED, false);
        name.setGravity(Gravity.CENTER);
        TextView data = text(value, 13, TEXT, true);
        data.setGravity(Gravity.CENTER);
        content.addView(name, matchHeight(dp(12)));
        content.addView(data, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1));
        batteryBox.addView(content, new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT));

        View border = new View(this);
        border.setBackground(round(Color.TRANSPARENT, BORDER, 18));
        batteryBox.addView(border, new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT));
        row.addView(batteryBox, halfParams(first));
        return data;
    }

    private void setBatteryProgress(int percent) {
        if (batteryBox == null || batteryFill == null) return;
        int bounded = Math.max(0, Math.min(100, percent));
        Runnable update = () -> {
            int width = batteryBox.getWidth();
            if (width <= 0) return;
            FrameLayout.LayoutParams lp =
                    (FrameLayout.LayoutParams) batteryFill.getLayoutParams();
            lp.width = Math.round(width * bounded / 100f);
            batteryFill.setLayoutParams(lp);
        };
        if (batteryBox.getWidth() > 0) {
            update.run();
        } else {
            batteryBox.post(update);
        }
    }

    private LinearLayout.LayoutParams halfParams(boolean first) {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(0,
                ViewGroup.LayoutParams.MATCH_PARENT, 1);
        if (first) lp.setMargins(0, 0, dp(4), 0);
        return lp;
    }

    private TextView sensorBadge(String label) {
        TextView badge = text("● " + label, 10, Color.rgb(255, 139, 144), true);
        badge.setTag(label);
        badge.setGravity(Gravity.CENTER);
        badge.setBackground(round(Color.rgb(78, 34, 38), RED, 18));
        return badge;
    }

    private void addSensor(LinearLayout row, TextView badge, boolean first) {
        row.addView(badge, halfParams(first));
    }

    private TextView text(String value, int sp, int color, boolean bold) {
        TextView view = new TextView(this);
        view.setText(value);
        view.setTextSize(sp);
        view.setTextColor(color);
        view.setGravity(Gravity.CENTER_VERTICAL);
        if (bold) view.setTypeface(view.getTypeface(), android.graphics.Typeface.BOLD);
        return view;
    }

    private Button button(String label, int color, int sp) {
        Button button = new Button(this);
        button.setText(label);
        button.setTextSize(sp);
        button.setTextColor(Color.WHITE);
        button.setAllCaps(false);
        button.setPadding(dp(4), 0, dp(4), 0);
        button.setBackground(round(color, color, 18));
        return button;
    }

    private GradientDrawable round(int fill, int stroke, int radiusDp) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(fill);
        drawable.setCornerRadius(dp(radiusDp));
        drawable.setStroke(dp(1), stroke);
        return drawable;
    }

    private LinearLayout vertical() {
        LinearLayout layout = new LinearLayout(this);
        layout.setOrientation(LinearLayout.VERTICAL);
        return layout;
    }

    private LinearLayout horizontal() {
        LinearLayout layout = new LinearLayout(this);
        layout.setOrientation(LinearLayout.HORIZONTAL);
        return layout;
    }

    private LinearLayout.LayoutParams matchWrap() {
        return new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
    }

    private LinearLayout.LayoutParams matchHeight(int height) {
        return new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, height);
    }

    private LinearLayout.LayoutParams withMargins(LinearLayout.LayoutParams lp,
                                                  int left, int top, int right, int bottom) {
        lp.setMargins(dp(left), dp(top), dp(right), dp(bottom));
        return lp;
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private static int integer(String value) {
        return Integer.parseInt(value.trim());
    }

    private static float decimal(String value) {
        return Float.parseFloat(value.trim());
    }

    private static boolean validDistance(float value) {
        return Float.isFinite(value) && value > 0f && value < 50f;
    }
}
