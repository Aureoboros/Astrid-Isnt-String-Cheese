package org.firstinspires.ftc.teamcode.config;

import static org.junit.Assert.*;
import java.io.File;
import javax.xml.parsers.DocumentBuilderFactory;
import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public final class GateMotorConfigTest {
  @Test public void recordsRequestedNamePortAndRatingWithoutChangingShooter() {
    assertEquals("gateMotor", RobotConfig.Hardware.GATE_MOTOR);
    assertEquals(1, RobotConfig.GateMotor.EXPANSION_HUB_PORT);
    assertEquals(1500, RobotConfig.GateMotor.MAX_RPM, 0);
    assertEquals(200, RobotConfig.GateMotor.TEST_INITIAL_RPM, 0);
    assertEquals(4500, RobotConfig.Diagnostics.GATE_TEST_SHOOTER_RPM, 0);
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
