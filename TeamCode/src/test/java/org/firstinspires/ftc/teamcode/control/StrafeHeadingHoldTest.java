package org.firstinspires.ftc.teamcode.control;

import static org.junit.Assert.*;
import org.junit.Test;

public final class StrafeHeadingHoldTest {
  private StrafeHeadingHold controller() { return new StrafeHeadingHold(.03, .001, 0, .3, 20, 1); }
  private double rad(double degrees) { return Math.toRadians(degrees); }

  @Test public void capturesCurrentHeadingAndCorrectsObservedMinus5ToMinus19Drift() {
    StrafeHeadingHold c = controller();
    assertEquals(0,c.update(.6,0,rad(-5),0,true),0);
    assertEquals(-5,c.getTargetDegrees(),1e-9);
    assertTrue(c.update(.6,0,rad(-19),.1,true)>0);
    assertEquals(14,c.getErrorDegrees(),1e-9);
    assertTrue(c.getCorrection()<=.3);
  }
  @Test public void correctionSignDependsOnHeadingErrorNotStrafeDirection() {
    for(double strafe : new double[] {-.6,.6}) {
      StrafeHeadingHold c=controller(); c.update(strafe,0,0,0,true);
      assertTrue(c.update(strafe,0,rad(5),.1,true)<0);
      assertTrue(c.update(strafe,0,rad(-5),.2,true)>0);
    }
  }
  @Test public void smallTriggerInputOverridesImmediatelyAndReleaseRecaptures() {
    StrafeHeadingHold c=controller(); c.update(.6,0,0,0,true); c.update(.6,0,rad(-5),.1,true);
    assertEquals(.01,c.update(.6,.01,rad(-5),.2,true),0); assertFalse(c.isActive());
    assertEquals(0,c.getCorrection(),0);
    assertEquals(0,c.update(.6,0,rad(30),.3,true),0); assertEquals(30,c.getTargetDegrees(),1e-9);
  }
  @Test public void idleAndForwardOnlyReleaseHeadingRatherThanRotateInPlace() {
    StrafeHeadingHold c=controller(); c.update(.6,0,0,0,true);
    assertEquals(0,c.update(0,0,rad(-10),.1,true),0); assertFalse(c.isActive());
    c.update(-.6,0,rad(15),.2,true); assertEquals(15,c.getTargetDegrees(),1e-9);
  }
  @Test public void reversingStrafeWithoutReleaseKeepsOriginalTarget() {
    StrafeHeadingHold c=controller(); c.update(.6,0,rad(10),0,true);
    c.update(-.6,0,rad(12),.1,true); assertEquals(10,c.getTargetDegrees(),1e-9);
  }
  @Test public void wrapsHeadingAcrossMinus180Plus180ByShortestTurn() {
    StrafeHeadingHold c=controller(); c.update(.6,0,rad(179),0,true);
    assertTrue(c.update(.6,0,rad(-179),.1,true)<0); assertEquals(-2,c.getErrorDegrees(),1e-9);
  }
  @Test public void deadbandSuppressesSmallCorrectionsWithoutDroppingTarget() {
    StrafeHeadingHold c=controller(); c.update(.6,0,0,0,true);
    assertEquals(0,c.update(.6,0,rad(.5),.1,true),0); assertTrue(c.isActive());
  }
  @Test public void faultInvalidMeasurementAndExplicitResetDropOldTarget() {
    StrafeHeadingHold c=controller(); c.update(.6,0,0,0,true);
    assertEquals(0,c.update(.6,0,rad(5),.1,false),0); assertFalse(c.isActive());
    c.update(.6,0,0,.2,true); assertEquals(0,c.update(.6,0,Double.NaN,.3,true),0);
    assertFalse(c.isActive()); c.update(.6,0,rad(30),.4,true); c.reset();
    assertFalse(c.isActive()); assertTrue(Double.isNaN(c.getTargetDegrees()));
  }
  @Test public void derivativeWrapsAndLongLoopGapDoesNotGenerateDerivativeKick() {
    StrafeHeadingHold c=new StrafeHeadingHold(0,0,.001,.3,20,0);
    c.update(.6,0,0,0,true); c.update(.6,0,rad(-179),.1,true);
    assertEquals(.02,c.update(.6,0,rad(179),.2,true),1e-9);
    assertEquals(0,c.update(.6,0,rad(10),2,true),0);
  }
  @Test public void integralIsBoundedAndClearedWhenDriverOverrides() {
    StrafeHeadingHold c=new StrafeHeadingHold(0,.1,0,.3,1,0);
    c.update(.6,0,0,0,true);
    for(int i=1;i<=100;i++) c.update(.6,0,rad(-5),i*.1,true);
    assertEquals(.1,c.getCorrection(),1e-9);
    c.update(.6,.1,0,10.1,true); c.update(.6,0,0,10.2,true);
    assertEquals(0,c.getCorrection(),0);
  }
  @Test public void simpleYawSimulationReducesFourteenDegreeDriftInBothStrafeDirections() {
    for(double strafe : new double[] {-.6,.6}) {
      StrafeHeadingHold c=controller(); double heading=rad(-5);
      double disturbance=strafe>0?-7:7;
      c.update(strafe,0,heading,0,true);
      for(int i=1;i<=100;i++) {
        double turn=c.update(strafe,0,heading,i*.02,true);
        assertTrue(Math.abs(turn)<=.3);
        heading+=rad((disturbance+turn*100)*.02);
      }
      assertTrue(Math.abs(Math.toDegrees(heading)+5)<4);
    }
  }
}
