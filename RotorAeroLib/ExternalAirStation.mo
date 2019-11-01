within RotorAeroLib;

/*
COPYRIGHT (C) 2019
BY ATA ENGINEERING, INC.
ALL RIGHTS RESERVED
*/

model ExternalAirStation
  extends PartialAirStation;
  
equation
  [delta_Fy, delta_Fz, delta_Tx] = AirStation_external(r,v_rotor,theta,thetadot);
  
end ExternalAirStation;
