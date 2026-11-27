package org.firstinspires.ftc.teamcode.localization;
import static org.junit.Assert.*;
import org.junit.Test;
public final class AbsolutePoseResetTest {
  private final AbsolutePoseReset reset = new AbsolutePoseReset(5, 4, 2, 5, 72);
  private PoseEstimate pose(double x, double y, double h) { return new PoseEstimate(x,y,h); }
  private void sample(int frame, PoseEstimate odo, PoseEstimate vision) {
    reset.update(frame*.1, true, true, false, frame, odo, vision);
  }
  @Test public void resetsToAbsoluteMapPositionAndHeadingOnce() {
    reset.begin(0);
    for (int i=1;i<=4;i++) { sample(i,pose(0,0,0),pose(25,-15,90)); assertNull(reset.takeCorrection()); }
    sample(5,pose(0,0,0),pose(25,-15,90));
    PoseEstimate p=reset.takeCorrection(); assertEquals(25,p.getX(),1e-9); assertEquals(-15,p.getY(),1e-9);
    assertEquals(90,p.getHeadingDegrees(),1e-9); assertFalse(reset.isActive()); assertNull(reset.takeCorrection());
  }
  @Test public void duplicatePollsAndOlderFramesDoNotCount() {
    reset.begin(0); sample(1,pose(0,0,0),pose(20,10,0));
    for(int i=0;i<20;i++) sample(1,pose(0,0,0),pose(20,10,0));
    sample(0,pose(0,0,0),pose(20,10,0)); assertEquals(1,reset.getFrameCount()); assertNull(reset.takeCorrection());
  }
  @Test public void correctionAccountsForCreepAcrossSamples() {
    reset.begin(0);
    for(int i=1;i<=5;i++) sample(i,pose(i,0,0),pose(20+i,10,0));
    PoseEstimate p=reset.takeCorrection(); assertEquals(25,p.getX(),1e-9); assertEquals(10,p.getY(),1e-9);
  }
  @Test public void headingsAcrossWrapRemainNear180RatherThanZero() {
    reset.begin(0);
    for(int i=1;i<=5;i++) sample(i,pose(0,0,0),pose(0,0,i%2==0?179:-179));
    assertEquals(180,Math.abs(reset.takeCorrection().getHeadingDegrees()),1);
  }
  @Test public void mappedOriginCanReplaceNonzeroOdometry() {
    reset.begin(0); for(int i=1;i<=5;i++) sample(i,pose(20,15,60),pose(0,0,0));
    PoseEstimate p=reset.takeCorrection(); assertEquals(0,p.getX(),1e-9); assertEquals(0,p.getY(),1e-9);
    assertEquals(0,p.getHeadingDegrees(),1e-9);
  }
  @Test public void noisyPositionSamplesDoNotApplyReset() {
    reset.begin(0); for(int i=1;i<=5;i++) sample(i,pose(0,0,0),pose(i==5?35:20,0,0));
    assertFalse(reset.isActive()); assertNull(reset.takeCorrection());
  }
  @Test public void noisyHeadingSamplesDoNotApplyReset() {
    reset.begin(0); for(int i=1;i<=5;i++) sample(i,pose(0,0,0),pose(20,0,i==5?30:0));
    assertFalse(reset.isActive()); assertNull(reset.takeCorrection());
  }
  @Test public void driveInputClearsSamplesAndWaitsForRelease() {
    reset.begin(0); sample(1,pose(0,0,0),pose(20,10,0));
    reset.update(.2,true,true,true,2,pose(0,0,0),pose(20,10,0)); assertEquals(0,reset.getFrameCount());
    for(int i=3;i<=7;i++) sample(i,pose(0,0,0),pose(20,10,0)); assertNotNull(reset.takeCorrection());
  }
  @Test public void unreadyCameraDisconnectedOrUnmappedDoNotCollectAndTimeoutLeavesPoseAlone() {
    reset.begin(0);
    reset.update(.1,false,true,false,1,pose(0,0,0),pose(20,10,0));
    reset.update(.2,true,false,false,2,pose(0,0,0),pose(20,10,0));
    sample(3,pose(0,0,0),null); assertEquals(0,reset.getFrameCount());
    reset.update(4,true,true,false,4,pose(0,0,0),pose(20,10,0));
    assertFalse(reset.isActive()); assertNull(reset.takeCorrection());
  }
  @Test public void outOfFieldAndNonfinitePosesAreRejected() {
    reset.begin(0); sample(1,pose(0,0,0),pose(73,0,0));
    sample(2,pose(Double.NaN,0,0),pose(20,0,0));
    sample(3,pose(0,0,0),pose(20,0,Double.NaN)); assertEquals(0,reset.getFrameCount());
  }
  @Test public void unavailableRequestCannotReuseOldCorrection() {
    reset.begin(0); for(int i=1;i<=5;i++) sample(i,pose(0,0,0),pose(20,0,0));
    reset.unavailable("disconnected"); assertNull(reset.takeCorrection()); assertFalse(reset.isActive());
  }
}
