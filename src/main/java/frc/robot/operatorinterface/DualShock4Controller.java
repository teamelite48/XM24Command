package frc.robot.operatorinterface;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DualShock4Controller extends GenericHID {

  public DualShock4Controller(int controllerPort) {
    super(controllerPort);
  }

  @Override
  public double getX(Hand hand) {
    switch (hand) {
      case kLeft:
        return this.getRawAxis(ButtonMap.LeftXAxis);
      case kRight:
        return this.getRawAxis(ButtonMap.RightXAxis);
      default:
        return 0;
    }
  }

  @Override
  public double getY(Hand hand) {
    switch (hand) {
      case kLeft:
        return -this.getRawAxis(ButtonMap.LeftYAxis);
      case kRight:
        return -this.getRawAxis(ButtonMap.RightYAxis);
      default:
        return 0;
    }
  }

  public JoystickButton getLeftBumper() {
    return new JoystickButton(this, ButtonMap.LeftBumper);
  }

  public JoystickButton getRightBumper() {
    return new JoystickButton(this, ButtonMap.RightBumper);
  }

  public JoystickButton getLeftStick() {
    return new JoystickButton(this, ButtonMap.LeftStick);
  }

  public JoystickButton getXButton() {
    return new JoystickButton(this, ButtonMap.XButton);
  }

  public JoystickButton getTriangleButton() {
    return new JoystickButton(this, ButtonMap.TriangleButton);
  }

  public JoystickButton getCircleButton() {
    return new JoystickButton(this, ButtonMap.CircleButton);
  }

  private final class ButtonMap {
    public static final int LeftXAxis = 0;
    public static final int LeftYAxis = 1;
    public static final int LeftBumper = 5;
    public static final int RightBumper = 6;
    public static final int LeftStick = 10;
    public static final int RightXAxis = 4;
    public static final int RightYAxis = 5;
    public static final int XButton = 2;
    public static final int TriangleButton = 4;
    public static final int CircleButton = 3;
  }
}
