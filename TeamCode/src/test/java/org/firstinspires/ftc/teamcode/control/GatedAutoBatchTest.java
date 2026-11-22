package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.config.PreloadedHiveAutoConfig;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Exercises the shot, encoder-feed and jam controllers together with a simulated gate motor. */
public final class GatedAutoBatchTest {
  private static final class Result {
    HiveShotController shot;
    GateJamRecoveryController gate;
    double elapsed;
  }
  private Result run(boolean jamInitially, boolean permanentlyJammed, boolean rpmReady, boolean requireTip) {
    double ticksPerRev = RobotConfig.GateMotor.TICKS_PER_REVOLUTION;
    double rpm = PreloadedHiveAutoConfig.GATE_RPM;
    GateJamRecoveryController gate = new GateJamRecoveryController(ticksPerRev,.4,.5,.2,.1,.15,.1,60,2,5,3);
    GateFeedProgress progress = new GateFeedProgress(ticksPerRev,rpm);
    HiveShotController shot = new HiveShotController(0,.15,4,requireTip,
        PreloadedHiveAutoConfig.SHOOT_BATCH_TIMEOUT_SECONDS);
    double position = 0, virtual = 0, previousOutput = 0;
    boolean wasFeeding = false;
    Result result = new Result(); result.shot = shot; result.gate = gate;
    for (int i = 1; i < 3000; i++) {
      double now = i*.01;
      boolean jammed = permanentlyJammed || (jamInitially && now < 1.8);
      double actualRpm = jammed ? 0 : previousOutput;
      position += actualRpm*ticksPerRev/60*.01;
      int ticks = (int)Math.round(position);
      gate.update(shot.feederOn()?rpm:0,actualRpm,ticks,now);
      if (gate.getState() == GateJamRecoveryController.State.FAULT) {
        result.elapsed = now; return result;
      }
      virtual += wasFeeding ? progress.update(ticks,
          gate.getState() == GateJamRecoveryController.State.RUNNING && Math.abs(actualRpm)>=rpm*.2,.01) : .01;
      boolean tipped = requireTip && shot.getCompletedPulses()==4
          && shot.getState()==HiveShotController.State.WAIT_TIP;
      shot.update(virtual,true,true,false,tipped,rpmReady,true);
      previousOutput = gate.update(shot.feederOn()?rpm:0,actualRpm,ticks,now);
      if (shot.feederOn() && !wasFeeding) { progress.start(ticks); }
      wasFeeding = shot.feederOn();
      if (shot.isDone()) { result.elapsed=now; return result; }
    }
    fail("Gated batch failed to finish or fault"); return result;
  }
  @Test public void threeLiveBatchesEstimateTwelveFeedsWithTipsOnFirstAndLast() {
    int feeds=0;
    for (boolean requireTip : new boolean[] {true,false,true}) {
      Result result=run(false,false,true,requireTip);
      assertEquals(HiveShotController.State.COMPLETE,result.shot.getState());
      assertEquals(4,result.shot.getCompletedPulses());
      assertEquals(requireTip,result.shot.isTipConfirmed());
      feeds+=result.shot.getCompletedPulses();
      assertEquals(0,result.gate.getOutputRpm(),0);
    }
    assertEquals(12,feeds);
  }
  @Test public void jamRecoveryExtendsFeedWithoutCreditingStalledGateTravel() {
    Result normal=run(false,false,true,false), recovered=run(true,false,true,false);
    assertEquals(HiveShotController.State.COMPLETE,recovered.shot.getState());
    assertEquals(4,recovered.shot.getCompletedPulses());
    assertTrue(recovered.gate.getRecoveryCount()>0);
    assertTrue(recovered.elapsed>normal.elapsed);
  }
  @Test public void persistentJamStopsBeforeAnyCompletedFeed() {
    Result result=run(false,true,true,false);
    assertEquals(GateJamRecoveryController.State.FAULT,result.gate.getState());
    assertEquals(3,result.gate.getRecoveryCount());
    assertEquals(0,result.shot.getCompletedPulses());
    assertEquals(0,result.gate.getOutputRpm(),0);
  }
  @Test public void shooterBelow2900ReadinessNeverStartsGate() {
    Result result=run(false,false,false,false);
    assertEquals(HiveShotController.State.FAILED,result.shot.getState());
    assertEquals(0,result.shot.getCompletedPulses());
    assertEquals(0,result.gate.getRecoveryCount());
    assertEquals(0,result.gate.getOutputRpm(),0);
  }
}
