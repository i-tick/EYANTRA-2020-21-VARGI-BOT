
"use strict";

let KitTray = require('./KitTray.js');
let Kit = require('./Kit.js');
let TrayContents = require('./TrayContents.js');
let LogicalCameraImage = require('./LogicalCameraImage.js');
let Order = require('./Order.js');
let VacuumGripperState = require('./VacuumGripperState.js');
let PopulationState = require('./PopulationState.js');
let Model = require('./Model.js');
let StorageUnit = require('./StorageUnit.js');
let KitObject = require('./KitObject.js');
let Proximity = require('./Proximity.js');
let ConveyorBeltState = require('./ConveyorBeltState.js');
let DetectedObject = require('./DetectedObject.js');

module.exports = {
  KitTray: KitTray,
  Kit: Kit,
  TrayContents: TrayContents,
  LogicalCameraImage: LogicalCameraImage,
  Order: Order,
  VacuumGripperState: VacuumGripperState,
  PopulationState: PopulationState,
  Model: Model,
  StorageUnit: StorageUnit,
  KitObject: KitObject,
  Proximity: Proximity,
  ConveyorBeltState: ConveyorBeltState,
  DetectedObject: DetectedObject,
};
