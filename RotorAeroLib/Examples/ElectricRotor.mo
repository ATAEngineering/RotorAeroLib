within RotorAeroLib.Examples;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model ElectricRotor
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
  import SI = Modelica.SIunits;
  parameter Boolean rigid = true annotation(Evaluate = true);
  parameter Integer N = 6 "Blade discretization" annotation(Evaluate = true);
  parameter Boolean orientation_ccw = true "Rotor orientation (=true for counter-clockwise)"   annotation(Evaluate = true);
  parameter Real RPM_guess = 720 "Rotor RPM initial guess";
  parameter SI.Velocity v_freestream = 0.0 "Free-stream velocity";
  parameter Real[3] n_freestream = {1, 0, 0} "Free-stream velocity direction (in world coordinates)";
  parameter Real shaft_length = 0.08898774 "Length of shaft from gimbal to blades(m)";
  parameter Real twist75 = 35.0 "Twist at 3/4 span (deg) (for setting collective shift)";
  Real AdvRatio;
  //
  inner RotorAeroLib.Thrust thrust(N_blade = 4, R = 0.25719, R_cutout = 0.0707, blade_chord = 0.05557, mu = 0.99, n_freestream = n_freestream, rho(displayUnit = "kg/m3") = 1.1596, useInflowCorrection = false, useMachCorrection = false, useTipLossCorrection = false, useUnsteadyAero = false, useUnsteadyNoncircAero = false, v_freestream = v_freestream) annotation(
    Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Voltage annotation(
    Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute drive(a(fixed = true), n = {1, 0, 0}, phi(fixed = true), stateSelect = StateSelect.always, useAxisFlange = true, w(start = RPM_guess / 60 * 2 * Modelica.Constants.pi)) annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RotorAeroLib.RotorBladeAssembly bladeAssm(N = N, numBlade = 4, orientation_ccw = orientation_ccw, rigid = rigid, shaft_length = 0) annotation(
    Placement(visible = true, transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body pylonMass(I_11 = 1e-10, I_22 = 1e-10, I_33 = 1e-10, animation = false, m = 1e-10) annotation(
    Placement(visible = true, transformation(origin = {88, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant collectiveSource(k = twist75) annotation(
    Placement(visible = true, transformation(origin = {60, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute driveFollower(animation = false, n = {1, 0, 0}, useAxisFlange = true) annotation(
    Placement(visible = true, transformation(origin = {-4, -32}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor1 annotation(
    Placement(visible = true, transformation(origin = {32, -54}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Position position1(exact = true, useSupport = true) annotation(
    Placement(visible = true, transformation(origin = {-16, -68}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation shaft(r = {shaft_length, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet motor(IaNominal = dcpmData1.IaNominal, Jr = dcpmData1.Jr, Js = dcpmData1.Js, La = dcpmData1.La, Ra = dcpmData1.Ra, TaNominal = dcpmData1.TaNominal, TaOperational = 293.15, TaRef = dcpmData1.TaRef, VaNominal = dcpmData1.VaNominal, alpha20a = dcpmData1.alpha20a, useSupport = true, wNominal = dcpmData1.wNominal) annotation(
    Placement(visible = true, transformation(extent = {{-2, 24}, {18, 44}}, rotation = 0)));
  parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData dcpmData1 annotation(
    Placement(visible = true, transformation(extent = {{60, 60}, {80, 80}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation(
    Placement(visible = true, transformation(origin = {-66, 80}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
    Placement(visible = true, transformation(origin = {-66, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bladeAssm.frame_a, drive.frame_b) annotation(
    Line(points = {{80, 0}, {60, 0}}, color = {95, 95, 95}));
  connect(collectiveSource.y, bladeAssm.collective) annotation(
    Line(points = {{71, -84}, {78, -84}, {78, -8}}, color = {0, 0, 127}));
  connect(driveFollower.frame_b, bladeAssm.relativeFrame_resolve) annotation(
    Line(points = {{6, -32}, {72, -32}, {72, 8}, {80, 8}}, color = {95, 95, 95}));
  connect(Voltage, signalVoltage.v) annotation(
    Line(points = {{-120, 80}, {-74, 80}, {-74, 80}, {-74, 80}}, color = {0, 0, 127}));
  connect(signalVoltage.n, ground.p) annotation(
    Line(points = {{-66, 70}, {-66, 70}, {-66, 36}, {-66, 36}}, color = {0, 0, 255}));
  connect(motor.pin_an, signalVoltage.n) annotation(
    Line(points = {{2, 44}, {-66, 44}, {-66, 70}, {-66, 70}}, color = {0, 0, 255}));
  connect(signalVoltage.p, motor.pin_ap) annotation(
    Line(points = {{-66, 90}, {14, 90}, {14, 44}, {14, 44}}, color = {0, 0, 255}));
  connect(motor.support, drive.support) annotation(
    Line(points = {{18, 24}, {44, 24}, {44, 10}}));
  connect(motor.flange, drive.axis) annotation(
    Line(points = {{18, 34}, {50, 34}, {50, 10}}));
  connect(driveFollower.frame_a, frame_a) annotation(
    Line(points = {{-14, -32}, {-100, -32}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(shaft.frame_a, frame_a) annotation(
    Line(points = {{4, 0}, {-100, 0}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(shaft.frame_b, drive.frame_a) annotation(
    Line(points = {{24, 0}, {40, 0}, {40, 0}, {40, 0}}, color = {95, 95, 95}));
  connect(angleSensor1.phi, position1.phi_ref) annotation(
    Line(points = {{20, -54}, {8, -54}, {8, -90}, {-28, -90}, {-28, -68}, {-28, -68}}, color = {0, 0, 127}));
  connect(position1.flange, driveFollower.axis) annotation(
    Line(points = {{-6, -68}, {-4, -68}, {-4, -42}, {-4, -42}}));
  connect(position1.support, driveFollower.support) annotation(
    Line(points = {{-16, -58}, {-10, -58}, {-10, -42}, {-10, -42}}));
  connect(angleSensor1.flange, drive.axis) annotation(
    Line(points = {{42, -54}, {50, -54}, {50, 10}, {50, 10}}));
  connect(pylonMass.frame_a, drive.frame_b) annotation(
    Line(points = {{78, 34}, {60, 34}, {60, 0}}, color = {95, 95, 95}));
  thrust.omega_in = drive.w;
  AdvRatio = -Modelica.Constants.pi * thrust.v_freestream / drive.w / thrust.R;
end ElectricRotor;