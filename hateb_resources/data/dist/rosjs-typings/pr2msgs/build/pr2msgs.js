var __extends = (this && this.__extends) || (function () {
    var extendStatics = Object.setPrototypeOf ||
        ({ __proto__: [] } instanceof Array && function (d, b) { d.__proto__ = b; }) ||
        function (d, b) { for (var p in b) if (b.hasOwnProperty(p)) d[p] = b[p]; };
    return function (d, b) {
        extendStatics(d, b);
        function __() { this.constructor = d; }
        d.prototype = b === null ? Object.create(b) : (__.prototype = b.prototype, new __());
    };
})();
var PR2Msgs;
(function (PR2Msgs) {
    var AccelerometerState = (function (_super) {
        __extends(AccelerometerState, _super);
        function AccelerometerState(values) {
            return _super.call(this, values) || this;
        }
        return AccelerometerState;
    }(ROSLIB.Message));
    PR2Msgs.AccelerometerState = AccelerometerState;
    var AccessPoint = (function (_super) {
        __extends(AccessPoint, _super);
        function AccessPoint(values) {
            return _super.call(this, values) || this;
        }
        return AccessPoint;
    }(ROSLIB.Message));
    PR2Msgs.AccessPoint = AccessPoint;
    var BatteryServer2 = (function (_super) {
        __extends(BatteryServer2, _super);
        function BatteryServer2(values) {
            var _this = _super.call(this, values) || this;
            _this.MAX_BAT_COUNT = 4;
            _this.MAX_BAT_REG = 48;
            return _this;
        }
        return BatteryServer2;
    }(ROSLIB.Message));
    PR2Msgs.BatteryServer2 = BatteryServer2;
    var BatteryState2 = (function (_super) {
        __extends(BatteryState2, _super);
        function BatteryState2(values) {
            return _super.call(this, values) || this;
        }
        return BatteryState2;
    }(ROSLIB.Message));
    PR2Msgs.BatteryState2 = BatteryState2;
    var DashboardState = (function (_super) {
        __extends(DashboardState, _super);
        function DashboardState(values) {
            return _super.call(this, values) || this;
        }
        return DashboardState;
    }(ROSLIB.Message));
    PR2Msgs.DashboardState = DashboardState;
    var GPUStatus = (function (_super) {
        __extends(GPUStatus, _super);
        function GPUStatus(values) {
            return _super.call(this, values) || this;
        }
        return GPUStatus;
    }(ROSLIB.Message));
    PR2Msgs.GPUStatus = GPUStatus;
    var LaserScannerSignal = (function (_super) {
        __extends(LaserScannerSignal, _super);
        function LaserScannerSignal(values) {
            return _super.call(this, values) || this;
        }
        return LaserScannerSignal;
    }(ROSLIB.Message));
    PR2Msgs.LaserScannerSignal = LaserScannerSignal;
    var LaserTrajCmd = (function (_super) {
        __extends(LaserTrajCmd, _super);
        function LaserTrajCmd(values) {
            return _super.call(this, values) || this;
        }
        return LaserTrajCmd;
    }(ROSLIB.Message));
    PR2Msgs.LaserTrajCmd = LaserTrajCmd;
    var PeriodicCmd = (function (_super) {
        __extends(PeriodicCmd, _super);
        function PeriodicCmd(values) {
            return _super.call(this, values) || this;
        }
        return PeriodicCmd;
    }(ROSLIB.Message));
    PR2Msgs.PeriodicCmd = PeriodicCmd;
    var MasterStateType;
    (function (MasterStateType) {
    })(MasterStateType = PR2Msgs.MasterStateType || (PR2Msgs.MasterStateType = {}));
    PR2Msgs.MASTER_NOPOWER = 0;
    PR2Msgs.MASTER_STANDBY = 1;
    PR2Msgs.MASTER_ON = 2;
    PR2Msgs.MASTER_OFF = 3;
    PR2Msgs.MASTER_SHUTDOWN = 4;
    var CircuitStateType;
    (function (CircuitStateType) {
    })(CircuitStateType = PR2Msgs.CircuitStateType || (PR2Msgs.CircuitStateType = {}));
    PR2Msgs.STATE_NOPOWER = 0;
    PR2Msgs.STATE_STANDBY = 1;
    PR2Msgs.STATE_PUMPING = 2;
    PR2Msgs.STATE_ON = 3;
    PR2Msgs.STATE_ENABLED = 3;
    PR2Msgs.STATE_DISABLED = 4;
    var PowerBoardState = (function (_super) {
        __extends(PowerBoardState, _super);
        function PowerBoardState(values) {
            return _super.call(this, values) || this;
        }
        return PowerBoardState;
    }(ROSLIB.Message));
    PR2Msgs.PowerBoardState = PowerBoardState;
    var PowerState = (function (_super) {
        __extends(PowerState, _super);
        function PowerState(values) {
            return _super.call(this, values) || this;
        }
        return PowerState;
    }(ROSLIB.Message));
    PR2Msgs.PowerState = PowerState;
    var PressureState = (function (_super) {
        __extends(PressureState, _super);
        function PressureState(values) {
            return _super.call(this, values) || this;
        }
        return PressureState;
    }(ROSLIB.Message));
    PR2Msgs.PressureState = PressureState;
    var SetLaserTrajCmdRequest = (function (_super) {
        __extends(SetLaserTrajCmdRequest, _super);
        function SetLaserTrajCmdRequest(values) {
            return _super.call(this, values) || this;
        }
        return SetLaserTrajCmdRequest;
    }(ROSLIB.ServiceRequest));
    PR2Msgs.SetLaserTrajCmdRequest = SetLaserTrajCmdRequest;
    var SetLaserTrajCmdResponse = (function (_super) {
        __extends(SetLaserTrajCmdResponse, _super);
        function SetLaserTrajCmdResponse(values) {
            return _super.call(this, values) || this;
        }
        return SetLaserTrajCmdResponse;
    }(ROSLIB.ServiceResponse));
    PR2Msgs.SetLaserTrajCmdResponse = SetLaserTrajCmdResponse;
    var SetPeriodicCmdRequest = (function (_super) {
        __extends(SetPeriodicCmdRequest, _super);
        function SetPeriodicCmdRequest(values) {
            return _super.call(this, values) || this;
        }
        return SetPeriodicCmdRequest;
    }(ROSLIB.ServiceRequest));
    PR2Msgs.SetPeriodicCmdRequest = SetPeriodicCmdRequest;
    var SetPeriodicCmdResponse = (function (_super) {
        __extends(SetPeriodicCmdResponse, _super);
        function SetPeriodicCmdResponse(values) {
            return _super.call(this, values) || this;
        }
        return SetPeriodicCmdResponse;
    }(ROSLIB.ServiceResponse));
    PR2Msgs.SetPeriodicCmdResponse = SetPeriodicCmdResponse;
})(PR2Msgs || (PR2Msgs = {}));
//# sourceMappingURL=pr2msgs.js.map