//
//  This command will set a triggering Event, and will end when that event has been processed.
//  Thus you can create a SerialCommandGroup full of triggeringEvents and they will execute one at a time.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.tower.TowerState;
import frc.robot.subsystems.tower.Tower;

public class WaitForTowerStateCmd extends Command {
  Tower tower;
  TowerState     state;

  /** Creates a new TriggerEvent. */
  public WaitForTowerStateCmd(Tower tower, TowerState state) {
    this.tower = tower;
    this.state = state;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (tower.getState() == state);
  }
}
