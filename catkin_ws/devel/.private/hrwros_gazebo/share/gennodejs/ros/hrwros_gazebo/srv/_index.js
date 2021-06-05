
"use strict";

let PopulationControl = require('./PopulationControl.js')
let GetMaterialLocations = require('./GetMaterialLocations.js')
let SubmitTray = require('./SubmitTray.js')
let ConveyorBeltControl = require('./ConveyorBeltControl.js')
let VacuumGripperControl = require('./VacuumGripperControl.js')
let AGVControl = require('./AGVControl.js')

module.exports = {
  PopulationControl: PopulationControl,
  GetMaterialLocations: GetMaterialLocations,
  SubmitTray: SubmitTray,
  ConveyorBeltControl: ConveyorBeltControl,
  VacuumGripperControl: VacuumGripperControl,
  AGVControl: AGVControl,
};
