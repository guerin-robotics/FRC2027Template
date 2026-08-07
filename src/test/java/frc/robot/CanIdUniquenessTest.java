// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.generated.TunerConstants;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

/**
 * Guards the CAN ID map against the failure that has no symptom.
 *
 * <p>Two devices sharing an ID is the worst bug on the bus: nothing errors, nothing logs, and the
 * mechanism half-works. The code commands ID 20 and two motors answer, or one device wins
 * arbitration and the other is simply never heard from. {@code .claude/rules/00-safety.md} lists
 * "wrong CAN ID — controls wrong motor; mechanism moves unexpectedly" first for a reason.
 *
 * <p>This test reads the two places IDs are allowed to live — {@link TunerConstants} for swerve and
 * {@link Constants.CanIds} for everything else — and cross-checks them. It needs no HAL, no
 * hardware, and no update as the season grows: adding a device to {@code CanIds} automatically
 * brings it under test.
 *
 * <p><b>What it cannot check.</b> Whether a constant matches the ID actually flashed into the
 * device. That lives in Phoenix Tuner X and in {@code docs/hardware-layout.md}, and it is the
 * reason the first-deploy checklist in {@code .claude/rules/04-build.md} asks for a Tuner device
 * count. This test only proves the map is internally consistent.
 */
class CanIdUniquenessTest {

  /**
   * The bus a {@link Constants.CanIds} entry is assumed to be on.
   *
   * <p>Reflection can read the number but not the {@code // RIO CAN} comment beside it, so this
   * relies on the rule in {@code .claude/rules/02-hardware.md}: swerve owns the CANivore and every
   * other mechanism is on the roboRIO bus. If a mechanism is ever deliberately placed on the
   * CANivore, that exception has to be taught to this test as well as written down.
   */
  private static final String MECHANISM_BUS = Constants.CanIds.RIO_BUS.getName();

  /**
   * Phoenix accepts device IDs 0 through 62. 63 is reserved for broadcast, and a negative ID is a
   * typo that constructs successfully and never talks to anything.
   */
  private static final int MIN_DEVICE_ID = 0;

  private static final int MAX_DEVICE_ID = 62;

  /** One CAN device: which bus it is on, its ID, and what to call it in a failure message. */
  private record CanDevice(String bus, int id, String owner) {
    @Override
    public String toString() {
      return owner + " (bus \"" + bus + "\", id " + id + ")";
    }
  }

  @Test
  void noTwoDevicesShareAnIdOnTheSameBus() {
    Map<String, List<CanDevice>> byBusAndId = new LinkedHashMap<>();
    for (CanDevice device : allDevices()) {
      byBusAndId
          .computeIfAbsent(device.bus() + "#" + device.id(), key -> new ArrayList<>())
          .add(device);
    }

    List<String> collisions =
        byBusAndId.values().stream()
            .filter(devices -> devices.size() > 1)
            .map(
                devices ->
                    devices.stream().map(CanDevice::toString).collect(Collectors.joining(" and ")))
            .toList();

    assertTrue(
        collisions.isEmpty(),
        () ->
            "Two devices are assigned the same CAN ID on the same bus:\n  "
                + String.join("\n  ", collisions)
                + "\n\nPhoenix only requires IDs to be unique per device TYPE — a TalonFX and a"
                + " CANcoder may legally share an ID. This repo is deliberately stricter, because"
                + " \"what is on ID 14?\" has to have one answer while someone is holding a wire."
                + " Renumber one of the devices and update docs/hardware-layout.md.");
  }

  @Test
  void everyIdIsInThePhoenixValidRange() {
    List<String> outOfRange =
        allDevices().stream()
            .filter(device -> device.id() < MIN_DEVICE_ID || device.id() > MAX_DEVICE_ID)
            .map(CanDevice::toString)
            .toList();

    assertTrue(
        outOfRange.isEmpty(),
        () ->
            "CAN IDs must be "
                + MIN_DEVICE_ID
                + "-"
                + MAX_DEVICE_ID
                + " (63 is the broadcast address):\n  "
                + String.join("\n  ", outOfRange));
  }

  @Test
  void mechanismIdsDoNotReuseTheSwerveBlock() {
    Set<Integer> swerveIds =
        swerveDevices().stream().map(CanDevice::id).collect(Collectors.toCollection(TreeSet::new));

    List<String> reused =
        mechanismDevices().stream()
            .filter(device -> swerveIds.contains(device.id()))
            .map(CanDevice::toString)
            .toList();

    assertTrue(
        reused.isEmpty(),
        () ->
            "These mechanism IDs reuse a number already assigned to swerve "
                + swerveIds
                + ":\n  "
                + String.join("\n  ", reused)
                + "\n\nThese are on different buses, so Phoenix would tolerate it. The convention"
                + " documented on Constants.CanIds — swerve owns the low block, mechanisms start"
                + " at 20 — exists so an ID read off a device in the pit identifies it without"
                + " also having to establish which bus it is on. If you are deliberately dropping"
                + " that convention, delete this test and the comment on Constants.CanIds"
                + " together.");
  }

  /** Every device the code knows about, swerve and mechanisms alike. */
  private static List<CanDevice> allDevices() {
    List<CanDevice> devices = new ArrayList<>(swerveDevices());
    devices.addAll(mechanismDevices());
    return devices;
  }

  /**
   * Swerve devices, read from the Tuner X output rather than restated here.
   *
   * <p>Reading the generated constants is what keeps this test correct across the regeneration that
   * {@code .claude/rules/02-hardware.md} requires for the 2027 robot — a hardcoded expectation here
   * would have to be hand-updated and would silently pass against the wrong numbers.
   */
  private static List<CanDevice> swerveDevices() {
    String bus = TunerConstants.kCANBus.getName();
    List<CanDevice> devices = new ArrayList<>();

    devices.add(new CanDevice(bus, TunerConstants.DrivetrainConstants.Pigeon2Id, "Pigeon2"));
    addModule(devices, bus, "FrontLeft", TunerConstants.FrontLeft);
    addModule(devices, bus, "FrontRight", TunerConstants.FrontRight);
    addModule(devices, bus, "BackLeft", TunerConstants.BackLeft);
    addModule(devices, bus, "BackRight", TunerConstants.BackRight);

    return devices;
  }

  private static void addModule(
      List<CanDevice> devices, String bus, String name, SwerveModuleConstants<?, ?, ?> constants) {
    devices.add(new CanDevice(bus, constants.DriveMotorId, name + " drive motor"));
    devices.add(new CanDevice(bus, constants.SteerMotorId, name + " steer motor"));
    devices.add(new CanDevice(bus, constants.EncoderId, name + " CANcoder"));
  }

  /**
   * Every {@code public static final int} on {@link Constants.CanIds}.
   *
   * <p>Reflection rather than an enumerated list, so a device added to that class is covered
   * without anyone remembering to add it here. Non-int fields — {@code RIO_BUS} — are skipped.
   */
  private static List<CanDevice> mechanismDevices() {
    List<CanDevice> devices = new ArrayList<>();
    for (Field field : Constants.CanIds.class.getDeclaredFields()) {
      boolean isConstant =
          Modifier.isPublic(field.getModifiers())
              && Modifier.isStatic(field.getModifiers())
              && Modifier.isFinal(field.getModifiers());
      if (!isConstant || field.getType() != int.class) {
        continue;
      }
      try {
        devices.add(
            new CanDevice(
                MECHANISM_BUS, field.getInt(null), "Constants.CanIds." + field.getName()));
      } catch (IllegalAccessException e) {
        throw new AssertionError("Could not read Constants.CanIds." + field.getName(), e);
      }
    }
    return devices;
  }
}
