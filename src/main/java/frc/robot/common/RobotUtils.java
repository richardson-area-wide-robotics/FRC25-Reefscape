package frc.robot.common;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.UtilityClass;

@UtilityClass
public class RobotUtils  {

  /**
   * Helper method to bind a control action to a command.
   *
   * @param control The button to bind to.
   * @param command The command to execute when that button is pressed.
   *
   * @author Hudson Strub
   * @since 2025
   */
  public static void bindControl(Trigger control, Command command, Command stopCommand) {
    control.whileTrue(command).whileFalse(stopCommand);
  }

  /**
   * Helper method to get the team number, the same as {@link HALUtil#getTeamNumber}
   * Only added because I can never remember the import
   *
   * @author Hudson Strub
   * @since 2025
   */
  public static int getTeamNumber(){
    return HALUtil.getTeamNumber();
  }

}