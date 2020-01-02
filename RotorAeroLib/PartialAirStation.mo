within RotorAeroLib;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model PartialAirStation
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_b;
  import SI = Modelica.SIunits;
  //
  parameter Boolean reverse_direction = false annotation(
    Evaluate = true);
  parameter Real orientation_factor = if reverse_direction then -1.0 else 1.0;
  parameter Real c = 0.121 "Chord length";
  parameter Real b = c / 2 "Semi-chord length";
  parameter Real a = 0.0 "Distance from to blade rotation position to mid-chord, measured in semi-chords";
  parameter Modelica.SIunits.Length delta_r = 1.0 "Effective length used to lump field forces/torques";
  //
  Modelica.Mechanics.MultiBody.Interfaces.Frame_resolve relativeFrame_resolve if resolveSensorInFrame == Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve "The sensor signals are optionally resolved in this frame" annotation(
    Placement(transformation(origin = {0, 100}, extent = {{16, -16}, {-16, 16}}, rotation = 270)));
  parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameA resolveSensorInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve "Frame in which sensor data (r, v, angles) is resolved (1: world, 2: frame_b, 3: frame_resolve)";
  parameter Modelica.Mechanics.MultiBody.Types.ResolveInFrameB resolveLoadInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve "Frame in which force data (force, torque) is resolved (1: world, 2: frame_b, 3: frame_resolve)";
  //
  SI.Force[3] f;
  SI.Torque[3] t;
  SI.Position[3] r;
  SI.Velocity[3] v_rotor;
  SI.Angle[3] angles;
  SI.Angle theta;
  Real thetadot;
  Real delta_Fx, delta_Fz, delta_Fy;
  Real delta_Tx, delta_Ty, delta_Tz;
  //
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteSensor absoluteSensor(animation = false, get_angles = false, get_r = false, get_v = true, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_resolve) annotation(
    HideResult = true,
    Placement(visible = true, transformation(origin = {0, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque force(animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(visible = true, transformation(origin = {0, -60}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.RelativeSensor relativeSensor(animation = false, get_angles = true, get_r_rel = true) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(absoluteSensor.frame_resolve, relativeFrame_resolve) annotation(
    Line(points = {{-10, 60}, {-26, 60}, {-26, 100}, {0, 100}, {0, 100}}, color = {95, 95, 95}));
  connect(force.frame_b, frame_b) annotation(
    Line(points = {{10, -60}, {47, -60}, {47, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(relativeSensor.frame_a, relativeFrame_resolve) annotation(
    Line(points = {{-10, 0}, {-48, 0}, {-48, 100}, {0, 100}, {0, 100}}, color = {95, 95, 95}));
  connect(relativeSensor.frame_b, frame_b) annotation(
    Line(points = {{10, 0}, {100, 0}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(absoluteSensor.frame_a, frame_b) annotation(
    Line(points = {{10, 60}, {55, 60}, {55, 0}, {100, 0}}, color = {95, 95, 95}));
//
  v_rotor = absoluteSensor.v;
//
  r = relativeSensor.r_rel;
//
  angles = relativeSensor.angles;
//
  theta = angles[1];
  thetadot = der(theta);
//
  force.force = f;
  force.torque = t;
//
  if reverse_direction then
    f[1] = delta_Fx;
    f[2] = -delta_Fy;
    f[3] = delta_Fz;
    t[1] = delta_Tx;
    t[2] = delta_Ty;
    t[3] = delta_Tz;
  else
    f[1] = delta_Fx;
    f[2] = delta_Fy;
    f[3] = delta_Fz;
    t[1] = delta_Tx;
    t[2] = delta_Ty;
    t[3] = delta_Tz;
  end if;
//
  annotation(
    Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}})}, coordinateSystem(initialScale = 0.1)));
end PartialAirStation;
