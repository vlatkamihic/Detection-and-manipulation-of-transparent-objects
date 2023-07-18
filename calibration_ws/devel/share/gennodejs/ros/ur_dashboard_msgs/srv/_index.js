
"use strict";

let IsProgramSaved = require('./IsProgramSaved.js')
let AddToLog = require('./AddToLog.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let Popup = require('./Popup.js')
let GetProgramState = require('./GetProgramState.js')
let GetRobotMode = require('./GetRobotMode.js')
let RawRequest = require('./RawRequest.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let Load = require('./Load.js')

module.exports = {
  IsProgramSaved: IsProgramSaved,
  AddToLog: AddToLog,
  GetSafetyMode: GetSafetyMode,
  IsProgramRunning: IsProgramRunning,
  Popup: Popup,
  GetProgramState: GetProgramState,
  GetRobotMode: GetRobotMode,
  RawRequest: RawRequest,
  GetLoadedProgram: GetLoadedProgram,
  IsInRemoteControl: IsInRemoteControl,
  Load: Load,
};
