package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.navigation.BiobuzzAutoRoutes.Plan;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.Alliance;
import org.firstinspires.ftc.teamcode.navigation.BiobuzzFieldMap.StartLocation;

/** Hardware-independent INIT selection; any selection change invalidates placement confirmation. */
public final class AutonomousStartControls {
  private Alliance alliance = Alliance.RED;
  private int startIndex;
  private Plan plan = Plan.POSE_ONLY;
  private boolean confirmed;
  private boolean oldLeft, oldRight, oldUp, oldDown, oldPreviousPlan, oldNextPlan, oldA;

  /** Returns true when a selection changes or the operator freshly confirms placement. */
  public boolean update(boolean left, boolean right, boolean up, boolean down,
      boolean previousPlan, boolean nextPlan, boolean a) {
    boolean changed = false;
    Alliance selectedAlliance = alliance;
    if (left && !oldLeft) {
      selectedAlliance = Alliance.RED;
    } else if (right && !oldRight) {
      selectedAlliance = Alliance.BLUE;
    }
    if (selectedAlliance != alliance) {
      alliance = selectedAlliance;
      startIndex = 0;
      changed = true;
    }
    int count = BiobuzzFieldMap.startsFor(alliance).size();
    if (up && !oldUp) {
      startIndex = (startIndex + 1) % count;
      changed = true;
    } else if (down && !oldDown) {
      startIndex = (startIndex + count - 1) % count;
      changed = true;
    }
    if (previousPlan && !oldPreviousPlan) {
      Plan[] plans = Plan.values();
      plan = plans[(plan.ordinal() + plans.length - 1) % plans.length];
      changed = true;
    } else if (nextPlan && !oldNextPlan) {
      Plan[] plans = Plan.values();
      plan = plans[(plan.ordinal() + 1) % plans.length];
      changed = true;
    }
    if (changed) {
      confirmed = false;
    }
    boolean confirm = a && !oldA && !changed;
    if (confirm) {
      confirmed = true;
    }
    oldLeft = left;
    oldRight = right;
    oldUp = up;
    oldDown = down;
    oldPreviousPlan = previousPlan;
    oldNextPlan = nextPlan;
    oldA = a;
    return changed || confirm;
  }

  public Alliance getAlliance() { return alliance; }
  public StartLocation getStart() { return BiobuzzFieldMap.startsFor(alliance).get(startIndex); }
  public Plan getPlan() { return plan; }
  public boolean isConfirmed() { return confirmed; }
}
