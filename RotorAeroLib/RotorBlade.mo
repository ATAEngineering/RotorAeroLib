within RotorAeroLib;

model RotorBlade
  /*
  COPYRIGHT (C) 2019
  BY ATA ENGINEERING, INC.
  ALL RIGHTS RESERVED
  */
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialOneFrame_a;
  parameter Integer N = 1 "Blade discretization/Number of AirStations";
  parameter Boolean rigid = true;
  parameter Boolean orientation_ccw = true "Rotor orientation (=true for counter-clockwise)" annotation(Evaluate = true);
  parameter Real orientation_factor = if orientation_ccw then 1.0 else -1.0;
  parameter Real Cl_0 = 0.0;
  parameter Real Cl_alpha = 6.28;
  parameter Real Cl_alpha2 = 0.0;
  parameter Real Cd_0 = 0.01;
  parameter Real Cd_alpha = 0.0;
  parameter Real Cd_alpha2 = 0.0;
  parameter Real radius = 48.0 * 0.0254 "(m)";
  parameter Real root_cutout = 0.15625 * radius "(m)";
  parameter Real collective_offset = orientation_factor * 4.67 "Shift for zero lift offset";
  parameter Real bearing1_dist = root_cutout / 2 "(m)";
  parameter Real chord = 6.0 * 0.0254 "(m)";
  parameter Boolean useLumpedMassMatrix = true;
  parameter Real beta = 0.01 "Blade material damping coefficient";
  parameter Real quarterChord_shift2 = 0.0;
  parameter Real delta_r = (radius - root_cutout) / N "AirStation blade span";
  parameter Real twist_total = orientation_factor * (-35.0) "Total twist from cutout to tip (deg)";
  parameter Real twist_rate = twist_total / (radius - root_cutout) "Twist rate (deg/R)";
  parameter Real twist_angle = twist_total / N "Twist per station (deg)";
  parameter Real twist_angle_rad = twist_angle * Modelica.Constants.pi / 180 "Twist per station(rad)";
  parameter Real collective_shift = (-twist_rate * (0.75 * radius - root_cutout)) + collective_offset "Shift to account for blade twist down to cutout";
  parameter Real blade_density = 853.472;
  parameter Real collective_shift_rad = collective_shift * Modelica.Constants.pi / 180;
  parameter Real shaft_length = 0.08898774 "Distance from hub to blade root (m)";
  Modelica.Mechanics.MultiBody.Interfaces.Frame_resolve relativeFrame_resolve annotation(
    Placement(visible = true, transformation(origin = {-100, 80}, extent = {{16, -16}, {-16, 16}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{16, -16}, {-16, 16}}, rotation = 0)));
  parameter DeployStructLib.Properties.EAGJ_BeamProperty Xsection2(EA = 1.17E+07, EIyy = 250.00, EIzz = 5200.00, GJ = 160.00, MOIxx = 7.18E-04, MOIyy = 1.70E-05, MOIzz = 7.01E-04, beta = beta, cen_y = 0.0, cen_z = 0.0, rCM_y = 0.0, rhoA = 1.071) annotation(
    Placement(visible = true, transformation(origin = {-6, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  DeployStructLib.Parts.Beam[M] beam1(L = {bearing1_dist, root_cutout - bearing1_dist}, each EAGJprop = Xsection1, each rigid = rigid, each useEAGJ = true, each useLumpedMassMatrix = useLumpedMassMatrix, each xprop = xprop1, each matProp = matProp1) annotation(
    Placement(visible = true, transformation(origin = {-26, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  DeployStructLib.Parts.Beam[N] beam2(each EAGJprop = Xsection2, each L = delta_r, each rigid = rigid, each useEAGJ = true, each useLumpedMassMatrix = useLumpedMassMatrix, each xprop = xprop2, each matProp = matProp2) annotation(
    Placement(visible = true, transformation(origin = {62, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body1(I_11 = 1e-10, I_22 = 1e-10, I_33 = 1e-10, animation = false, m = 1e-10, r_CM = {0, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {160, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation[N] quarterChordFrame2(each animation = true, each angle = twist_angle / 2, each r = {delta_r / 2, quarterChord_shift2, 0}) annotation(
    Placement(visible = true, transformation(origin = {-14, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation[N] bladeTwistFrame2(each animation = false, each angle = twist_angle, each r = {0, quarterChord_shift2 * (1 - cos(twist_angle_rad)), -quarterChord_shift2 * sin(twist_angle_rad)}) annotation(
    Placement(visible = true, transformation(origin = {106, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute collectiveShiftRevolute(animation = false, n = {1, 0, 0}, useAxisFlange = true) annotation(
    Placement(visible = true, transformation(origin = {-56, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation collectiveShift(animation = false, angle = collective_shift, r = {0.0, 0.0, 0.0}) annotation(
    Placement(visible = true, transformation(origin = {-56, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //
  AirStation[N] liftInterface2(each Cd0 = Cd_0, each Cd_alpha = Cd_alpha, each Cd_alpha2 = Cd_alpha2, each Cl0 = Cl_0, each Cl_alpha = Cl_alpha, each Cl_alpha2 = Cl_alpha2, each c = chord, each delta_r = delta_r, each reverse_direction = not orientation_ccw) annotation(
    Placement(visible = true, transformation(origin = {30, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //
  parameter DeployStructLib.Properties.BeamXProperties.RectBarProperty xprop1(height = chord / 4, width = chord / 4) annotation(
    Placement(visible = true, transformation(origin = {36, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter DeployStructLib.Properties.BeamXProperties.RectBarProperty xprop2(height = chord / 4, width = chord) annotation(
    Placement(visible = true, transformation(origin = {36, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter DeployStructLib.Properties.MaterialProperties.isotropicMaterialProperty matProp1(rho(displayUnit = "kg/m3") = blade_density) annotation(
    Placement(visible = true, transformation(origin = {76, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter DeployStructLib.Properties.MaterialProperties.isotropicMaterialProperty matProp2(rho(displayUnit = "kg/m3") = blade_density) annotation(
    Placement(visible = true, transformation(origin = {76, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter DeployStructLib.Properties.EAGJ_BeamProperty Xsection1(EA = 1.17E+07, EIyy = 250.00, EIzz = 5200.00, GJ = 160.00, MOIxx = 7.18E-04, MOIyy = 1.70E-05, MOIzz = 7.01E-04, beta = beta, cen_y = 0.0, cen_z = 0.0, rCM_y = 0.0, rhoA = 1.071) annotation(
    Placement(visible = true, transformation(origin = {-6, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput collective annotation(
    Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Position position1(exact = true, useSupport = true) annotation(
    Placement(visible = true, transformation(origin = {-66, -58}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation bladeShaft(animation = true, r = {shaft_length, 0, 0}) annotation(
    Placement(visible = true, transformation(origin = {-82, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //
protected
  final parameter Integer M = 2 annotation(Evaluate = true);
  //
equation
  connect(collectiveShift.frame_b, beam1[1].frame_a) annotation(
    Line(points = {{-46, 8}, {-41, 8}, {-41, 0}, {-36, 0}}));
  connect(collectiveShiftRevolute.frame_b, collectiveShift.frame_a) annotation(
    Line(points = {{-46, 0}, {-41, 0}, {-41, 8}, {-66, 8}}));
  position1.phi_ref = orientation_factor * collective * Modelica.Constants.pi / 180;
  connect(collectiveShiftRevolute.support, position1.support) annotation(
    Line(points = {{-62, -10}, {-78, -10}, {-78, -48}, {-66, -48}, {-66, -48}}));
  connect(position1.flange, collectiveShiftRevolute.axis) annotation(
    Line(points = {{-56, -58}, {-36, -58}, {-36, -10}, {-56, -10}, {-56, -10}}));
  connect(beam1[1].frame_b, beam1[2].frame_a) annotation(
    Line);
  connect(beam1[M].frame_b, beam2[1].frame_a) annotation(
    Line(points = {{18, -2}, {52, -2}}, color = {95, 95, 95}));
  connect(beam2[N].frame_b, bladeTwistFrame2[N].frame_a) annotation(
    Line);
  connect(bladeTwistFrame2[N].frame_b, body1.frame_a) annotation(
    Line(points = {{116, -2}, {150, -2}}, color = {95, 95, 95}));
  connect(liftInterface2[1].relativeFrame_resolve, relativeFrame_resolve) annotation(
    Line);
  connect(quarterChordFrame2[1].frame_b, liftInterface2[1].frame_b) annotation(
    Line);
  connect(beam2[1].frame_a, quarterChordFrame2[1].frame_a) annotation(
    Line);
  for i in 1:N - 1 loop
    connect(beam2[i].frame_b, bladeTwistFrame2[i].frame_a);
    connect(bladeTwistFrame2[i].frame_b, beam2[i + 1].frame_a);
    connect(bladeTwistFrame2[i].frame_b, quarterChordFrame2[i + 1].frame_a);
    connect(quarterChordFrame2[i + 1].frame_b, liftInterface2[i + 1].frame_b);
    connect(liftInterface2[i + 1].relativeFrame_resolve, relativeFrame_resolve);
  end for;
  connect(bladeShaft.frame_b, collectiveShiftRevolute.frame_a) annotation(
    Line(points = {{-72, 0}, {-66, 0}, {-66, 0}, {-66, 0}}, color = {95, 95, 95}));
  connect(bladeShaft.frame_a, frame_a) annotation(
    Line(points = {{-92, 0}, {-98, 0}, {-98, 0}, {-100, 0}}, color = {95, 95, 95}));
end RotorBlade;