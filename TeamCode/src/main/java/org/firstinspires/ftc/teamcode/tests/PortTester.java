package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Port Tester - Tests motors and servos one by one to verify wiring
 * 
 * CONTROLS:
 * - D-pad Up/Down: Select hub (Control Hub / Expansion Hub / Servo Hub)
 * - D-pad Left/Right: Select port (0-3 for motors, 0-5 for servos)
 * - Left Stick Y: Run motor (when motor port selected)
 * - Right Stick Y: Move servo (when servo hub selected)
 * - A: Toggle motor direction
 * - B: Center servo (0.5)
 * - X: Min servo (0.0)
 * - Y: Max servo (1.0)
 */
@TeleOp(name = "Port Tester", group = "Tuning")
public class PortTester extends OpMode {

    private enum Hub {
        CONTROL_HUB("Control Hub"),
        EXPANSION_HUB("Expansion Hub"),
        SERVO_HUB("Servo Hub");

        final String name;
        Hub(String name) { this.name = name; }
    }

    // State
    private Hub selectedHub = Hub.CONTROL_HUB;
    private int selectedPort = 0;
    private boolean motorReversed = false;
    private double servoPosition = 0.5;

    // Hardware - generic by port
    private DcMotorEx[] controlHubMotors = new DcMotorEx[4];
    private DcMotorEx[] expansionHubMotors = new DcMotorEx[4];
    private Servo[] servos = new Servo[6];

    // Telemetry
    private MultipleTelemetry multiTelemetry;

    // Debounce for button presses
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void init() {
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Control Hub motors (ports 0-3)
        String[] controlHubNames = {"FL", "FR", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            try {
                controlHubMotors[i] = hardwareMap.get(DcMotorEx.class, controlHubNames[i]);
                controlHubMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                controlHubMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                controlHubMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (Exception e) {
                controlHubMotors[i] = null;
            }
        }

        // Initialize Expansion Hub motors (ports 0-3)
        String[] expansionHubNames = {"turret", "flywheel", "flywheel2", "intake"};
        for (int i = 0; i < 4; i++) {
            try {
                expansionHubMotors[i] = hardwareMap.get(DcMotorEx.class, expansionHubNames[i]);
                expansionHubMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                expansionHubMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                expansionHubMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (Exception e) {
                expansionHubMotors[i] = null;
            }
        }

        // Initialize Servo Hub servos (ports 0-5)
        String[] servoNames = {"hood", "hood2", "spindexer", "transfer", "servo4", "servo5"};
        for (int i = 0; i < 6; i++) {
            try {
                servos[i] = hardwareMap.get(Servo.class, servoNames[i]);
            } catch (Exception e) {
                servos[i] = null;
            }
        }

        multiTelemetry.addLine("Port Tester Ready");
        multiTelemetry.addLine("D-pad: Select Hub/Port | Sticks: Run Motor/Servo");
        multiTelemetry.update();
    }

    @Override
    public void loop() {
        // --- Input Handling ---
        
        // Hub selection (D-pad Up/Down)
        if (gamepad1.dpad_up && !lastDpadUp) {
            selectedHub = Hub.values()[(selectedHub.ordinal() + Hub.values().length - 1) % Hub.values().length];
            selectedPort = 0;
            motorReversed = false;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            selectedHub = Hub.values()[(selectedHub.ordinal() + 1) % Hub.values().length];
            selectedPort = 0;
            motorReversed = false;
        }

        // Port selection (D-pad Left/Right)
        int maxPort = (selectedHub == Hub.SERVO_HUB) ? 5 : 3;
        if (gamepad1.dpad_right && !lastDpadRight) {
            selectedPort = (selectedPort + 1) % (maxPort + 1);
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            selectedPort = (selectedPort + maxPort) % (maxPort + 1);
        }

        // Motor direction toggle (A button)
        if (gamepad1.a && !lastA) {
            motorReversed = !motorReversed;
        }

        // Servo presets
        if (gamepad1.b && !lastB) servoPosition = 0.5;
        if (gamepad1.x && !lastX) servoPosition = 0.0;
        if (gamepad1.y && !lastY) servoPosition = 1.0;

        // Update debounce states
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        // --- Motor/Servo Control ---
        
        // Stop all motors first
        for (DcMotorEx motor : controlHubMotors) {
            if (motor != null) motor.setPower(0);
        }
        for (DcMotorEx motor : expansionHubMotors) {
            if (motor != null) motor.setPower(0);
        }

        // Control based on selected hub
        double motorPower = -gamepad1.left_stick_y; // Invert for intuitive control
        if (motorReversed) motorPower = -motorPower;

        String deviceStatus = "NOT FOUND";
        int encoderValue = 0;

        if (selectedHub == Hub.CONTROL_HUB) {
            DcMotorEx motor = controlHubMotors[selectedPort];
            if (motor != null) {
                motor.setPower(motorPower);
                encoderValue = motor.getCurrentPosition();
                deviceStatus = String.format("Power: %.2f | Enc: %d", motorPower, encoderValue);
            }
        } else if (selectedHub == Hub.EXPANSION_HUB) {
            DcMotorEx motor = expansionHubMotors[selectedPort];
            if (motor != null) {
                motor.setPower(motorPower);
                encoderValue = motor.getCurrentPosition();
                deviceStatus = String.format("Power: %.2f | Enc: %d", motorPower, encoderValue);
            }
        } else if (selectedHub == Hub.SERVO_HUB) {
            // Continuous servo control with right stick
            servoPosition += -gamepad1.right_stick_y * 0.01;
            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
            
            Servo servo = servos[selectedPort];
            if (servo != null) {
                servo.setPosition(servoPosition);
                deviceStatus = String.format("Position: %.3f", servoPosition);
            }
        }

        // --- Telemetry ---
        multiTelemetry.addLine("=== PORT TESTER ===");
        multiTelemetry.addLine("");
        
        // Hub selection display
        for (Hub hub : Hub.values()) {
            String marker = (hub == selectedHub) ? ">> " : "   ";
            multiTelemetry.addLine(marker + hub.name);
        }
        multiTelemetry.addLine("");

        // Port selection display
        multiTelemetry.addData("Selected Port", selectedPort);
        multiTelemetry.addData("Status", deviceStatus);
        
        if (selectedHub != Hub.SERVO_HUB) {
            multiTelemetry.addData("Direction", motorReversed ? "REVERSED" : "FORWARD");
        }
        
        multiTelemetry.addLine("");
        multiTelemetry.addLine("--- CONTROLS ---");
        multiTelemetry.addLine("D-pad Up/Down: Change Hub");
        multiTelemetry.addLine("D-pad Left/Right: Change Port");
        if (selectedHub == Hub.SERVO_HUB) {
            multiTelemetry.addLine("Right Stick Y: Move Servo");
            multiTelemetry.addLine("X=0.0 | B=0.5 | Y=1.0");
        } else {
            multiTelemetry.addLine("Left Stick Y: Run Motor");
            multiTelemetry.addLine("A: Toggle Direction");
        }

        multiTelemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors
        for (DcMotorEx motor : controlHubMotors) {
            if (motor != null) motor.setPower(0);
        }
        for (DcMotorEx motor : expansionHubMotors) {
            if (motor != null) motor.setPower(0);
        }
    }
}
