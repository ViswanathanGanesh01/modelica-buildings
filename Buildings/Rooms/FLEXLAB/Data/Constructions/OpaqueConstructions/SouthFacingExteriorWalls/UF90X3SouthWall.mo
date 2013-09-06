within Buildings.Rooms.FLEXLAB.Data.Constructions.OpaqueConstructions.SouthFacingExteriorWalls;
record UF90X3SouthWall =
  Buildings.HeatTransfer.Data.OpaqueConstructions.Generic(final material={
    Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.02413),
    Buildings.HeatTransfer.Data.Solids.Plywood(x=0.0127),
    Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.08255),
    Buildings.HeatTransfer.Data.Solids.GypsumBoard(x=0.01588)},
    final nLay = 4) "South facing wall in test cell UF90X3. 24 mm 
     of rigid insulation, 13 mm of plywood, 83 mm of rigid insulation, 16 mm of gypsum 
     wallboard";
