within RotorAeroLib;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model AirStation
  extends PartialAirStation;
  import SI = Modelica.SIunits;
  //
  parameter Real Cl0 = 0.0;
  parameter Real Cl_alpha = 5.7;
  parameter Real Cl_alpha2 = 0.0;
  parameter Real Cd0 = 0.01;
  parameter Real Cd_alpha = 0.0;
  parameter Real Cd_alpha2 = 0.0;
  //
  SI.Velocity[3] v_freestream_rotor;
  SI.Acceleration a_rotor3 if thrust.useUnsteadyAero;
  Real U_perp(start = 1.0);
  Real U3(start = 1.0);
  Real U_inflow;
  Real U_tan, U, U2;
  SI.Angle alpha;
  Real Cl, Cd, delta_L, delta_D;
  Real Ma "Mach number";
  Real AspectRatio;
  Real Ma_factor, AspectRatioFactor, tipLossFactor;
  Real V_nondim if thrust.useUnsteadyAero "Non-dimensional velocity";
  Real unsteady_x1(start = 0.0) if thrust.useUnsteadyAero;
  Real unsteady_x2(start = 0.0, fixed = true) if thrust.useUnsteadyAero;
  Real unsteady_x2dot(start = 0.0, fixed = true) if thrust.useUnsteadyAero;
  Real Cl_noncirc if thrust.useUnsteadyAero;
  Real Cl_circ if thrust.useUnsteadyAero;
  Real lambda(start = 0.15);
  Real lambda_c;
  //
  outer Thrust thrust;
equation
//
  v_freestream_rotor = Modelica.Mechanics.MultiBody.Frames.resolve2(relativeFrame_resolve.R, thrust.v_freestream * thrust.n_freestream);
//
  if thrust.useInflowCorrection then
    lambda_c = abs(v_rotor[3] - v_freestream_rotor[3]) / thrust.R / abs(thrust.omega);
    lambda = homotopy(sqrt(abs((thrust.sigma * Cl_alpha / 16 - lambda_c / 2) ^ 2 + thrust.sigma * Cl_alpha / 8 * orientation_factor * theta * r[1] / thrust.R)) - (thrust.sigma * Cl_alpha / 16 - lambda_c / 2), v_freestream_rotor[3] / thrust.Vtip);
//
    U_inflow = lambda * thrust.Vtip;
    U_perp = v_rotor[3] + U_inflow;
//    
  else
    lambda_c = 0.0;
    lambda = 0.0;
    U_inflow = 0.0;
    U_perp = v_rotor[3] - v_freestream_rotor[3];
  end if;
//
  if reverse_direction then
    U_tan = v_freestream_rotor[2] - v_rotor[2];
  else
    U_tan = v_rotor[2] - v_freestream_rotor[2];
  end if;
  U = sqrt(U_tan ^ 2 + U_perp ^ 2);
// Rotor frame quantities
  U2 = U_tan * cos(orientation_factor * theta) + U_perp * sin(orientation_factor * theta);
  U3 = (-U_tan * sin(orientation_factor * theta)) + U_perp * cos(orientation_factor * theta);
//
// By convention
  alpha = atan(-U3 / U2);
//
//  Include unsteady aerodynamics here
//  This is based on state-space circulatory terms of Wagner's function (R.T. Jones' fit)
//  as detailed in Leishman's Principles of Helicopter Aerodynamics (pg. 464).
//
  if thrust.useUnsteadyAero then
    V_nondim = 2 * U / c;
    der(unsteady_x1) = unsteady_x2;
    der(unsteady_x2) = (-0.01375 * V_nondim ^ 2 * unsteady_x1) - 0.3455 * V_nondim * unsteady_x2 + alpha;
    unsteady_x2dot = der(unsteady_x2);
    Cl_circ = Cl_alpha * (0.006825 * V_nondim ^ 2 * unsteady_x1 + 0.10805 * V_nondim * unsteady_x2) + 0.25 * Cl_alpha * alpha;
    if thrust.useUnsteadyNoncircAero then
      a_rotor3 = der(v_rotor[3]);
      // The full calculation can be expensive/troublesome, so ignoring the higher order term as an approximation is the option (switch what is commented out here as necessary)
//      Cl_noncirc = Modelica.Constants.pi*b*(orientation_factor*thetadot/U + a_rotor3/U^2 + orientation_factor*b*a*der(thetadot)/(2*U^2));
      Cl_noncirc = Modelica.Constants.pi * b * (orientation_factor * thetadot / U + a_rotor3 / U ^ 2);
    else
      a_rotor3 = 0.0;
      Cl_noncirc = 0.0;
    end if;
    Cl = homotopy(Cl_noncirc + Cl_circ, Cl0 + Cl_alpha * alpha + Cl_alpha2 * alpha ^ 2);
  else
    Cl = Cl0 + Cl_alpha * alpha + Cl_alpha2 * alpha ^ 2;
  end if;
  Cd = Cd0 + Cd_alpha * alpha + Cd_alpha2 * alpha ^ 2;
//
  if thrust.useMachCorrection then
    Ma = U / thrust.c;
    Ma_factor = 1 / sqrt(1 - Ma ^ 2);
  else
    Ma = 0.0;
    Ma_factor = 1.0;
  end if;
//
  AspectRatio = 0.75 * thrust.R / c;
  if thrust.useAspectRatioCorrection then
    AspectRatioFactor = AspectRatio / (AspectRatio + 2);
  else
    AspectRatioFactor = 1.0;
  end if;
//
  if thrust.useTipLossCorrection then
    tipLossFactor = tanh((1 - r[1] / thrust.R) / (1 - thrust.mu));
  else
    tipLossFactor = 1.0;
  end if;
//
  delta_L = Ma_factor * AspectRatioFactor * tipLossFactor * 0.5 * thrust.rho * U ^ 2 * c * Cl * delta_r;
  delta_D = 0.5 * thrust.rho * U ^ 2 * c * Cd * delta_r;
//
  delta_Fz = delta_L * cos(alpha) + delta_D * sin(alpha);
  delta_Fy = delta_L * sin(alpha) - delta_D * cos(alpha);
  delta_Tx = 0;
//
  annotation(
    Icon(graphics = {Polygon(origin = {-8, 7}, rotation = 10, fillColor = {150, 150, 150}, fillPattern = FillPattern.Solid, points = {{-76, -13}, {-12, 11}, {32, 13}, {70, 3}, {76, -13}, {-76, -13}}), Line(origin = {0.74, -25.32}, points = {{0, -19}, {0, 19}}, thickness = 0.5, arrow = {Arrow.None, Arrow.Open}, arrowSize = 5), Line(origin = {25.56, -20.36}, points = {{0, -19}, {0, 19}}, thickness = 0.5, arrow = {Arrow.None, Arrow.Open}, arrowSize = 5), Line(origin = {52.27, -15.41}, points = {{0, -19}, {0, 19}}, thickness = 0.5, arrow = {Arrow.None, Arrow.Open}, arrowSize = 5), Line(origin = {-25.7, -29.68}, points = {{0, -19}, {0, 19}}, thickness = 0.5, arrow = {Arrow.None, Arrow.Open}, arrowSize = 5), Line(origin = {-51.96, -34.19}, points = {{0, -19}, {0, 19}}, thickness = 0.5, arrow = {Arrow.None, Arrow.Open}, arrowSize = 5), Rectangle(extent = {{-100, 100}, {100, -100}})}));
end AirStation;
