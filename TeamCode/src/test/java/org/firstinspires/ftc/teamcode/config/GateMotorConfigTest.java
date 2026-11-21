package org.firstinspires.ftc.teamcode.config;

import static org.junit.Assert.*;
import java.io.File;
import javax.xml.parsers.DocumentBuilderFactory;
import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public final class GateMotorConfigTest {
  @Test public void automaticFeedRemainsBlockedUntilPhysicalCalibration() {
    assertFalse(RobotConfig.GateMotor.AUTO_FEED_ENABLED);
    assertFalse(RobotConfig.GateMotor.ONE_BALL_INDEX_VERIFIED);
    assertFalse(RobotConfig.GateMotor.START_BLOCK_POSITION_VERIFIED);
    assertEquals(0, RobotConfig.GateMotor.TICKS_PER_INDEX);
    assertFalse(RobotConfig.GateMotor.autoFeedReady());
  }

  @Test public void recordsRequestedNamePortAndRatingWithoutChangingShooter() {
    assertEquals("gateMotor", RobotConfig.Hardware.GATE_MOTOR);
    assertEquals(1, RobotConfig.GateMotor.EXPANSION_HUB_PORT);
    assertEquals(435, RobotConfig.GateMotor.MAX_RPM, 0);
    assertEquals(384.5, RobotConfig.GateMotor.TICKS_PER_REVOLUTION, 0);
    assertEquals(100, RobotConfig.GateMotor.TEST_INITIAL_RPM, 0);
    assertEquals(25, RobotConfig.GateMotor.TEST_RPM_STEP, 0);
    assertEquals("Shoot1", RobotConfig.Hardware.SHOOTER_1);
    assertEquals(6000, RobotConfig.ShooterMotor.MAX_RPM, 0);
  }

  @Test public void referenceXmlAssignsGateOnceAndReplacesObsoleteOdometryPort() throws Exception {
    File xml = new File("../.run/RobotConfig.xml");
    if (!xml.isFile()) { xml = new File(".run/RobotConfig.xml"); }
    Document document = DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(xml);
    NodeList modules = document.getElementsByTagName("LynxModule");
    int gates = 0;
    for (int i = 0; i < modules.getLength(); i++) {
      Element module = (Element) modules.item(i);
      NodeList devices = module.getElementsByTagName("*");
      for (int j = 0; j < devices.getLength(); j++) {
        Element device = (Element) devices.item(j);
        assertNotEquals("yOdo", device.getAttribute("name"));
        if (RobotConfig.Hardware.GATE_MOTOR.equals(device.getAttribute("name"))) {
          gates++;
          assertEquals("Expansion Hub 2", module.getAttribute("name"));
          assertEquals("1", device.getAttribute("port"));
          assertEquals("Motor", device.getTagName()); // Do not guess gearing/encoder metadata.
        }
        if (RobotConfig.Hardware.SHOOTER_1.equals(device.getAttribute("name"))) {
          assertEquals("2", device.getAttribute("port"));
          assertEquals("GoBilda6000RpmMotor", device.getTagName());
        }
      }
    }
    assertEquals(1, gates);
  }
}
