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
var ActionLibMsgs;
(function (ActionLibMsgs) {
    var GoalStatusType;
    (function (GoalStatusType) {
    })(GoalStatusType = ActionLibMsgs.GoalStatusType || (ActionLibMsgs.GoalStatusType = {}));
    ActionLibMsgs.PENDING = 0;
    ActionLibMsgs.ACTIVE = 1;
    ActionLibMsgs.PREEMPTED = 2;
    ActionLibMsgs.SUCCEEDED = 3;
    ActionLibMsgs.ABORTED = 4;
    ActionLibMsgs.REJECTED = 5;
    ActionLibMsgs.PREEMPTING = 6;
    ActionLibMsgs.RECALLING = 7;
    ActionLibMsgs.RECALLED = 8;
    ActionLibMsgs.LOST = 9;
    var GoalStatus = (function (_super) {
        __extends(GoalStatus, _super);
        function GoalStatus(values) {
            return _super.call(this, values) || this;
        }
        return GoalStatus;
    }(ROSLIB.Message));
    ActionLibMsgs.GoalStatus = GoalStatus;
    var GoalStatusArray = (function (_super) {
        __extends(GoalStatusArray, _super);
        function GoalStatusArray(values) {
            return _super.call(this, values) || this;
        }
        return GoalStatusArray;
    }(ROSLIB.Message));
    ActionLibMsgs.GoalStatusArray = GoalStatusArray;
    var GoalID = (function (_super) {
        __extends(GoalID, _super);
        function GoalID(values) {
            return _super.call(this, values) || this;
        }
        return GoalID;
    }(ROSLIB.Message));
    ActionLibMsgs.GoalID = GoalID;
})(ActionLibMsgs || (ActionLibMsgs = {}));
var DynamicReconfigure;
(function (DynamicReconfigure) {
    var BoolParameter = (function (_super) {
        __extends(BoolParameter, _super);
        function BoolParameter(values) {
            return _super.call(this, values) || this;
        }
        return BoolParameter;
    }(ROSLIB.Message));
    DynamicReconfigure.BoolParameter = BoolParameter;
    var Config = (function (_super) {
        __extends(Config, _super);
        function Config(values) {
            return _super.call(this, values) || this;
        }
        return Config;
    }(ROSLIB.Message));
    DynamicReconfigure.Config = Config;
    var ConfigDescription = (function (_super) {
        __extends(ConfigDescription, _super);
        function ConfigDescription(values) {
            return _super.call(this, values) || this;
        }
        return ConfigDescription;
    }(ROSLIB.Message));
    DynamicReconfigure.ConfigDescription = ConfigDescription;
    var DoubleParameter = (function (_super) {
        __extends(DoubleParameter, _super);
        function DoubleParameter(values) {
            return _super.call(this, values) || this;
        }
        return DoubleParameter;
    }(ROSLIB.Message));
    DynamicReconfigure.DoubleParameter = DoubleParameter;
    var Group = (function (_super) {
        __extends(Group, _super);
        function Group(values) {
            return _super.call(this, values) || this;
        }
        return Group;
    }(ROSLIB.Message));
    DynamicReconfigure.Group = Group;
    var GroupState = (function (_super) {
        __extends(GroupState, _super);
        function GroupState(values) {
            return _super.call(this, values) || this;
        }
        return GroupState;
    }(ROSLIB.Message));
    DynamicReconfigure.GroupState = GroupState;
    var IntParameter = (function (_super) {
        __extends(IntParameter, _super);
        function IntParameter(values) {
            return _super.call(this, values) || this;
        }
        return IntParameter;
    }(ROSLIB.Message));
    DynamicReconfigure.IntParameter = IntParameter;
    var ParamDescription = (function (_super) {
        __extends(ParamDescription, _super);
        function ParamDescription(values) {
            return _super.call(this, values) || this;
        }
        return ParamDescription;
    }(ROSLIB.Message));
    DynamicReconfigure.ParamDescription = ParamDescription;
    var StrParameter = (function (_super) {
        __extends(StrParameter, _super);
        function StrParameter(values) {
            return _super.call(this, values) || this;
        }
        return StrParameter;
    }(ROSLIB.Message));
    DynamicReconfigure.StrParameter = StrParameter;
    var ReconfigureRequest = (function (_super) {
        __extends(ReconfigureRequest, _super);
        function ReconfigureRequest(values) {
            return _super.call(this, values) || this;
        }
        return ReconfigureRequest;
    }(ROSLIB.ServiceRequest));
    DynamicReconfigure.ReconfigureRequest = ReconfigureRequest;
    var ReconfigureResponse = (function (_super) {
        __extends(ReconfigureResponse, _super);
        function ReconfigureResponse(values) {
            return _super.call(this, values) || this;
        }
        return ReconfigureResponse;
    }(ROSLIB.ServiceRequest));
    DynamicReconfigure.ReconfigureResponse = ReconfigureResponse;
})(DynamicReconfigure || (DynamicReconfigure = {}));
var GeometryMsgs;
(function (GeometryMsgs) {
    var Accel = (function (_super) {
        __extends(Accel, _super);
        function Accel(values) {
            return _super.call(this, values) || this;
        }
        return Accel;
    }(ROSLIB.Message));
    GeometryMsgs.Accel = Accel;
    var AccelStamped = (function (_super) {
        __extends(AccelStamped, _super);
        function AccelStamped(values) {
            return _super.call(this, values) || this;
        }
        return AccelStamped;
    }(ROSLIB.Message));
    GeometryMsgs.AccelStamped = AccelStamped;
    var AccelWithCovariance = (function (_super) {
        __extends(AccelWithCovariance, _super);
        function AccelWithCovariance(values) {
            return _super.call(this, values) || this;
        }
        return AccelWithCovariance;
    }(ROSLIB.Message));
    GeometryMsgs.AccelWithCovariance = AccelWithCovariance;
    var AccelWithCovarianceStamped = (function (_super) {
        __extends(AccelWithCovarianceStamped, _super);
        function AccelWithCovarianceStamped(values) {
            return _super.call(this, values) || this;
        }
        return AccelWithCovarianceStamped;
    }(ROSLIB.Message));
    GeometryMsgs.AccelWithCovarianceStamped = AccelWithCovarianceStamped;
    var Inertia = (function (_super) {
        __extends(Inertia, _super);
        function Inertia(values) {
            return _super.call(this, values) || this;
        }
        return Inertia;
    }(ROSLIB.Message));
    GeometryMsgs.Inertia = Inertia;
    var InertiaStamped = (function (_super) {
        __extends(InertiaStamped, _super);
        function InertiaStamped(values) {
            return _super.call(this, values) || this;
        }
        return InertiaStamped;
    }(ROSLIB.Message));
    GeometryMsgs.InertiaStamped = InertiaStamped;
    var Point = (function (_super) {
        __extends(Point, _super);
        function Point(values) {
            return _super.call(this, values) || this;
        }
        return Point;
    }(ROSLIB.Message));
    GeometryMsgs.Point = Point;
    var Point32 = (function (_super) {
        __extends(Point32, _super);
        function Point32() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Point32;
    }(Point));
    GeometryMsgs.Point32 = Point32;
    var PointStamped = (function (_super) {
        __extends(PointStamped, _super);
        function PointStamped(values) {
            return _super.call(this, values) || this;
        }
        return PointStamped;
    }(ROSLIB.Message));
    GeometryMsgs.PointStamped = PointStamped;
    var Polygon = (function (_super) {
        __extends(Polygon, _super);
        function Polygon(values) {
            return _super.call(this, values) || this;
        }
        return Polygon;
    }(ROSLIB.Message));
    GeometryMsgs.Polygon = Polygon;
    var PolygonStamped = (function (_super) {
        __extends(PolygonStamped, _super);
        function PolygonStamped(values) {
            return _super.call(this, values) || this;
        }
        return PolygonStamped;
    }(ROSLIB.Message));
    GeometryMsgs.PolygonStamped = PolygonStamped;
    var Pose = (function (_super) {
        __extends(Pose, _super);
        function Pose(values) {
            return _super.call(this, values) || this;
        }
        return Pose;
    }(ROSLIB.Message));
    GeometryMsgs.Pose = Pose;
    var Pose2D = (function (_super) {
        __extends(Pose2D, _super);
        function Pose2D(values) {
            return _super.call(this, values) || this;
        }
        return Pose2D;
    }(ROSLIB.Message));
    GeometryMsgs.Pose2D = Pose2D;
    var PoseArray = (function (_super) {
        __extends(PoseArray, _super);
        function PoseArray(values) {
            return _super.call(this, values) || this;
        }
        return PoseArray;
    }(ROSLIB.Message));
    GeometryMsgs.PoseArray = PoseArray;
    var PoseStamped = (function (_super) {
        __extends(PoseStamped, _super);
        function PoseStamped(values) {
            return _super.call(this, values) || this;
        }
        return PoseStamped;
    }(ROSLIB.Message));
    GeometryMsgs.PoseStamped = PoseStamped;
    var PoseWithCovariance = (function (_super) {
        __extends(PoseWithCovariance, _super);
        function PoseWithCovariance(values) {
            return _super.call(this, values) || this;
        }
        return PoseWithCovariance;
    }(ROSLIB.Message));
    GeometryMsgs.PoseWithCovariance = PoseWithCovariance;
    var PoseWithCovarianceStamped = (function (_super) {
        __extends(PoseWithCovarianceStamped, _super);
        function PoseWithCovarianceStamped(values) {
            return _super.call(this, values) || this;
        }
        return PoseWithCovarianceStamped;
    }(ROSLIB.Message));
    GeometryMsgs.PoseWithCovarianceStamped = PoseWithCovarianceStamped;
    var Quaternion = (function (_super) {
        __extends(Quaternion, _super);
        function Quaternion(values) {
            return _super.call(this, values) || this;
        }
        return Quaternion;
    }(ROSLIB.Message));
    GeometryMsgs.Quaternion = Quaternion;
    var QuaternionStamped = (function (_super) {
        __extends(QuaternionStamped, _super);
        function QuaternionStamped(values) {
            return _super.call(this, values) || this;
        }
        return QuaternionStamped;
    }(ROSLIB.Message));
    GeometryMsgs.QuaternionStamped = QuaternionStamped;
    var Transform = (function (_super) {
        __extends(Transform, _super);
        function Transform(values) {
            return _super.call(this, values) || this;
        }
        return Transform;
    }(ROSLIB.Message));
    GeometryMsgs.Transform = Transform;
    var TransformStamped = (function (_super) {
        __extends(TransformStamped, _super);
        function TransformStamped(values) {
            return _super.call(this, values) || this;
        }
        return TransformStamped;
    }(ROSLIB.Message));
    GeometryMsgs.TransformStamped = TransformStamped;
    var Twist = (function (_super) {
        __extends(Twist, _super);
        function Twist(values) {
            return _super.call(this, values) || this;
        }
        return Twist;
    }(ROSLIB.Message));
    GeometryMsgs.Twist = Twist;
    var TwistStamped = (function (_super) {
        __extends(TwistStamped, _super);
        function TwistStamped(values) {
            return _super.call(this, values) || this;
        }
        return TwistStamped;
    }(ROSLIB.Message));
    GeometryMsgs.TwistStamped = TwistStamped;
    var TwistWithCovariance = (function (_super) {
        __extends(TwistWithCovariance, _super);
        function TwistWithCovariance(values) {
            return _super.call(this, values) || this;
        }
        return TwistWithCovariance;
    }(ROSLIB.Message));
    GeometryMsgs.TwistWithCovariance = TwistWithCovariance;
    var TwistWithCovarianceStamped = (function (_super) {
        __extends(TwistWithCovarianceStamped, _super);
        function TwistWithCovarianceStamped(values) {
            return _super.call(this, values) || this;
        }
        return TwistWithCovarianceStamped;
    }(ROSLIB.Message));
    GeometryMsgs.TwistWithCovarianceStamped = TwistWithCovarianceStamped;
    var Vector3 = (function (_super) {
        __extends(Vector3, _super);
        function Vector3() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Vector3;
    }(Point));
    GeometryMsgs.Vector3 = Vector3;
    var Vector3Stamped = (function (_super) {
        __extends(Vector3Stamped, _super);
        function Vector3Stamped(values) {
            return _super.call(this, values) || this;
        }
        return Vector3Stamped;
    }(ROSLIB.Message));
    GeometryMsgs.Vector3Stamped = Vector3Stamped;
    var Wrench = (function (_super) {
        __extends(Wrench, _super);
        function Wrench(values) {
            return _super.call(this, values) || this;
        }
        return Wrench;
    }(ROSLIB.Message));
    GeometryMsgs.Wrench = Wrench;
    var WrenchStamped = (function (_super) {
        __extends(WrenchStamped, _super);
        function WrenchStamped(values) {
            return _super.call(this, values) || this;
        }
        return WrenchStamped;
    }(ROSLIB.Message));
    GeometryMsgs.WrenchStamped = WrenchStamped;
})(GeometryMsgs || (GeometryMsgs = {}));
var HANPMsgs;
(function (HANPMsgs) {
    var HumanPath = (function (_super) {
        __extends(HumanPath, _super);
        function HumanPath(values) {
            return _super.call(this, values) || this;
        }
        return HumanPath;
    }(ROSLIB.Message));
    HANPMsgs.HumanPath = HumanPath;
    var HumanPathArray = (function (_super) {
        __extends(HumanPathArray, _super);
        function HumanPathArray(values) {
            return _super.call(this, values) || this;
        }
        return HumanPathArray;
    }(ROSLIB.Message));
    HANPMsgs.HumanPathArray = HumanPathArray;
    var HumanTrajectory = (function (_super) {
        __extends(HumanTrajectory, _super);
        function HumanTrajectory(values) {
            return _super.call(this, values) || this;
        }
        return HumanTrajectory;
    }(ROSLIB.Message));
    HANPMsgs.HumanTrajectory = HumanTrajectory;
    var HumanTrajectoryArray = (function (_super) {
        __extends(HumanTrajectoryArray, _super);
        function HumanTrajectoryArray(values) {
            return _super.call(this, values) || this;
        }
        return HumanTrajectoryArray;
    }(ROSLIB.Message));
    HANPMsgs.HumanTrajectoryArray = HumanTrajectoryArray;
    var TrackedHuman = (function (_super) {
        __extends(TrackedHuman, _super);
        function TrackedHuman(values) {
            return _super.call(this, values) || this;
        }
        return TrackedHuman;
    }(ROSLIB.Message));
    HANPMsgs.TrackedHuman = TrackedHuman;
    var TrackedHumans = (function (_super) {
        __extends(TrackedHumans, _super);
        function TrackedHumans(values) {
            return _super.call(this, values) || this;
        }
        return TrackedHumans;
    }(ROSLIB.Message));
    HANPMsgs.TrackedHumans = TrackedHumans;
    var TrackedSegment = (function (_super) {
        __extends(TrackedSegment, _super);
        function TrackedSegment(values) {
            return _super.call(this, values) || this;
        }
        return TrackedSegment;
    }(ROSLIB.Message));
    HANPMsgs.TrackedSegment = TrackedSegment;
    var TrackedSegmentType;
    (function (TrackedSegmentType) {
    })(TrackedSegmentType = HANPMsgs.TrackedSegmentType || (HANPMsgs.TrackedSegmentType = {}));
    HANPMsgs.HEAD = 0;
    HANPMsgs.TORSO = 1;
    HANPMsgs.RIGHT_SHOULDER = 2;
    HANPMsgs.RIGHT_ELBOW = 3;
    HANPMsgs.RIGHT_WRIST = 4;
    HANPMsgs.RIGHT_HIP = 5;
    HANPMsgs.RIGHT_KNEE = 6;
    HANPMsgs.RIGHT_ANKLE = 7;
    HANPMsgs.LEFT_SHOULDER = 8;
    HANPMsgs.LEFT_ELBOW = 9;
    HANPMsgs.LEFT_WRIST = 10;
    HANPMsgs.LEFT_HIP = 11;
    HANPMsgs.LEFT_KNEE = 12;
    HANPMsgs.LEFT_ANKLE = 13;
    var Trajectory = (function (_super) {
        __extends(Trajectory, _super);
        function Trajectory(values) {
            return _super.call(this, values) || this;
        }
        return Trajectory;
    }(ROSLIB.Message));
    HANPMsgs.Trajectory = Trajectory;
    var TrajectoryArray = (function (_super) {
        __extends(TrajectoryArray, _super);
        function TrajectoryArray(values) {
            return _super.call(this, values) || this;
        }
        return TrajectoryArray;
    }(ROSLIB.Message));
    HANPMsgs.TrajectoryArray = TrajectoryArray;
    var TrajectoryPoint = (function (_super) {
        __extends(TrajectoryPoint, _super);
        function TrajectoryPoint(values) {
            return _super.call(this, values) || this;
        }
        return TrajectoryPoint;
    }(ROSLIB.Message));
    HANPMsgs.TrajectoryPoint = TrajectoryPoint;
})(HANPMsgs || (HANPMsgs = {}));
var MoveBaseMsgs;
(function (MoveBaseMsgs) {
    var MoveBaseActionFeedback = (function (_super) {
        __extends(MoveBaseActionFeedback, _super);
        function MoveBaseActionFeedback(values) {
            return _super.call(this, values) || this;
        }
        return MoveBaseActionFeedback;
    }(ROSLIB.Message));
    MoveBaseMsgs.MoveBaseActionFeedback = MoveBaseActionFeedback;
    var MoveBaseActionGoal = (function (_super) {
        __extends(MoveBaseActionGoal, _super);
        function MoveBaseActionGoal(values) {
            return _super.call(this, values) || this;
        }
        return MoveBaseActionGoal;
    }(ROSLIB.Message));
    MoveBaseMsgs.MoveBaseActionGoal = MoveBaseActionGoal;
    var MoveBaseAction = (function (_super) {
        __extends(MoveBaseAction, _super);
        function MoveBaseAction(values) {
            return _super.call(this, values) || this;
        }
        return MoveBaseAction;
    }(ROSLIB.Message));
    MoveBaseMsgs.MoveBaseAction = MoveBaseAction;
    var MoveBaseActionResult = (function (_super) {
        __extends(MoveBaseActionResult, _super);
        function MoveBaseActionResult(values) {
            return _super.call(this, values) || this;
        }
        return MoveBaseActionResult;
    }(ROSLIB.Message));
    MoveBaseMsgs.MoveBaseActionResult = MoveBaseActionResult;
    var MoveBaseFeedback = (function (_super) {
        __extends(MoveBaseFeedback, _super);
        function MoveBaseFeedback(values) {
            return _super.call(this, values) || this;
        }
        return MoveBaseFeedback;
    }(ROSLIB.Message));
    MoveBaseMsgs.MoveBaseFeedback = MoveBaseFeedback;
    var MoveBaseGoal = (function (_super) {
        __extends(MoveBaseGoal, _super);
        function MoveBaseGoal(values) {
            return _super.call(this, values) || this;
        }
        return MoveBaseGoal;
    }(ROSLIB.Message));
    MoveBaseMsgs.MoveBaseGoal = MoveBaseGoal;
    var MoveBaseResult = (function (_super) {
        __extends(MoveBaseResult, _super);
        function MoveBaseResult() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return MoveBaseResult;
    }(ROSLIB.Message));
    MoveBaseMsgs.MoveBaseResult = MoveBaseResult;
})(MoveBaseMsgs || (MoveBaseMsgs = {}));
var NavMsgs;
(function (NavMsgs) {
    var GridCells = (function (_super) {
        __extends(GridCells, _super);
        function GridCells(values) {
            return _super.call(this, values) || this;
        }
        return GridCells;
    }(ROSLIB.Message));
    NavMsgs.GridCells = GridCells;
    var MapMetaData = (function (_super) {
        __extends(MapMetaData, _super);
        function MapMetaData(values) {
            return _super.call(this, values) || this;
        }
        return MapMetaData;
    }(ROSLIB.Message));
    NavMsgs.MapMetaData = MapMetaData;
    var OccupancyGrid = (function (_super) {
        __extends(OccupancyGrid, _super);
        function OccupancyGrid(values) {
            return _super.call(this, values) || this;
        }
        return OccupancyGrid;
    }(ROSLIB.Message));
    NavMsgs.OccupancyGrid = OccupancyGrid;
    var Odometry = (function (_super) {
        __extends(Odometry, _super);
        function Odometry(values) {
            return _super.call(this, values) || this;
        }
        return Odometry;
    }(ROSLIB.Message));
    NavMsgs.Odometry = Odometry;
    var Path = (function (_super) {
        __extends(Path, _super);
        function Path(values) {
            return _super.call(this, values) || this;
        }
        return Path;
    }(ROSLIB.Message));
    NavMsgs.Path = Path;
    var GetMapRequest = (function (_super) {
        __extends(GetMapRequest, _super);
        function GetMapRequest(values) {
            return _super.call(this, values) || this;
        }
        return GetMapRequest;
    }(ROSLIB.ServiceRequest));
    NavMsgs.GetMapRequest = GetMapRequest;
    var GetMapResponse = (function (_super) {
        __extends(GetMapResponse, _super);
        function GetMapResponse(values) {
            return _super.call(this, values) || this;
        }
        return GetMapResponse;
    }(ROSLIB.ServiceResponse));
    NavMsgs.GetMapResponse = GetMapResponse;
    var GetPlanRequest = (function (_super) {
        __extends(GetPlanRequest, _super);
        function GetPlanRequest(values) {
            return _super.call(this, values) || this;
        }
        return GetPlanRequest;
    }(ROSLIB.ServiceRequest));
    NavMsgs.GetPlanRequest = GetPlanRequest;
    var GetPlanResponse = (function (_super) {
        __extends(GetPlanResponse, _super);
        function GetPlanResponse(values) {
            return _super.call(this, values) || this;
        }
        return GetPlanResponse;
    }(ROSLIB.ServiceResponse));
    NavMsgs.GetPlanResponse = GetPlanResponse;
    var SetMapRequest = (function (_super) {
        __extends(SetMapRequest, _super);
        function SetMapRequest(values) {
            return _super.call(this, values) || this;
        }
        return SetMapRequest;
    }(ROSLIB.ServiceRequest));
    NavMsgs.SetMapRequest = SetMapRequest;
    var SetMapResponse = (function (_super) {
        __extends(SetMapResponse, _super);
        function SetMapResponse(values) {
            return _super.call(this, values) || this;
        }
        return SetMapResponse;
    }(ROSLIB.ServiceResponse));
    NavMsgs.SetMapResponse = SetMapResponse;
})(NavMsgs || (NavMsgs = {}));
var StdMsgs;
(function (StdMsgs) {
    var Bool = (function (_super) {
        __extends(Bool, _super);
        function Bool(values) {
            return _super.call(this, values) || this;
        }
        return Bool;
    }(ROSLIB.Message));
    StdMsgs.Bool = Bool;
    var Byte = (function (_super) {
        __extends(Byte, _super);
        function Byte(values) {
            return _super.call(this, values) || this;
        }
        return Byte;
    }(ROSLIB.Message));
    StdMsgs.Byte = Byte;
    var ByteMultiArray = (function (_super) {
        __extends(ByteMultiArray, _super);
        function ByteMultiArray(values) {
            return _super.call(this, values) || this;
        }
        return ByteMultiArray;
    }(ROSLIB.Message));
    StdMsgs.ByteMultiArray = ByteMultiArray;
    var Char = (function (_super) {
        __extends(Char, _super);
        function Char() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Char;
    }(Byte));
    StdMsgs.Char = Char;
    var ColorRGBA = (function (_super) {
        __extends(ColorRGBA, _super);
        function ColorRGBA(values) {
            return _super.call(this, values) || this;
        }
        return ColorRGBA;
    }(ROSLIB.Message));
    StdMsgs.ColorRGBA = ColorRGBA;
    var Duration = (function (_super) {
        __extends(Duration, _super);
        function Duration(values) {
            return _super.call(this, values) || this;
        }
        return Duration;
    }(ROSLIB.Message));
    StdMsgs.Duration = Duration;
    var Empty = (function (_super) {
        __extends(Empty, _super);
        function Empty(values) {
            return _super.call(this, values) || this;
        }
        return Empty;
    }(ROSLIB.Message));
    StdMsgs.Empty = Empty;
    var Float32 = (function (_super) {
        __extends(Float32, _super);
        function Float32() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Float32;
    }(Byte));
    StdMsgs.Float32 = Float32;
    var Float32MultiArray = (function (_super) {
        __extends(Float32MultiArray, _super);
        function Float32MultiArray() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Float32MultiArray;
    }(ByteMultiArray));
    StdMsgs.Float32MultiArray = Float32MultiArray;
    var Float64 = (function (_super) {
        __extends(Float64, _super);
        function Float64() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Float64;
    }(Byte));
    StdMsgs.Float64 = Float64;
    var Float64MultiArray = (function (_super) {
        __extends(Float64MultiArray, _super);
        function Float64MultiArray() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Float64MultiArray;
    }(ByteMultiArray));
    StdMsgs.Float64MultiArray = Float64MultiArray;
    var Header = (function (_super) {
        __extends(Header, _super);
        function Header(values) {
            return _super.call(this, values) || this;
        }
        return Header;
    }(ROSLIB.Message));
    StdMsgs.Header = Header;
    var Int16 = (function (_super) {
        __extends(Int16, _super);
        function Int16() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Int16;
    }(Byte));
    StdMsgs.Int16 = Int16;
    var Int16MultiArray = (function (_super) {
        __extends(Int16MultiArray, _super);
        function Int16MultiArray() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Int16MultiArray;
    }(ByteMultiArray));
    StdMsgs.Int16MultiArray = Int16MultiArray;
    var Int32 = (function (_super) {
        __extends(Int32, _super);
        function Int32() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Int32;
    }(Byte));
    StdMsgs.Int32 = Int32;
    var Int32MultiArray = (function (_super) {
        __extends(Int32MultiArray, _super);
        function Int32MultiArray() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Int32MultiArray;
    }(ByteMultiArray));
    StdMsgs.Int32MultiArray = Int32MultiArray;
    var Int64 = (function (_super) {
        __extends(Int64, _super);
        function Int64() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Int64;
    }(Byte));
    StdMsgs.Int64 = Int64;
    var Int64MultiArray = (function (_super) {
        __extends(Int64MultiArray, _super);
        function Int64MultiArray() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Int64MultiArray;
    }(ByteMultiArray));
    StdMsgs.Int64MultiArray = Int64MultiArray;
    var Int8 = (function (_super) {
        __extends(Int8, _super);
        function Int8() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Int8;
    }(Byte));
    StdMsgs.Int8 = Int8;
    var Int8MultiArray = (function (_super) {
        __extends(Int8MultiArray, _super);
        function Int8MultiArray() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return Int8MultiArray;
    }(ByteMultiArray));
    StdMsgs.Int8MultiArray = Int8MultiArray;
    var MultiArrayDimension = (function (_super) {
        __extends(MultiArrayDimension, _super);
        function MultiArrayDimension(values) {
            return _super.call(this, values) || this;
        }
        return MultiArrayDimension;
    }(ROSLIB.Message));
    StdMsgs.MultiArrayDimension = MultiArrayDimension;
    var MultiArrayLayout = (function (_super) {
        __extends(MultiArrayLayout, _super);
        function MultiArrayLayout(values) {
            return _super.call(this, values) || this;
        }
        return MultiArrayLayout;
    }(ROSLIB.Message));
    StdMsgs.MultiArrayLayout = MultiArrayLayout;
    var String = (function (_super) {
        __extends(String, _super);
        function String(values) {
            return _super.call(this, values) || this;
        }
        return String;
    }(ROSLIB.Message));
    StdMsgs.String = String;
    var Time = (function (_super) {
        __extends(Time, _super);
        function Time(values) {
            return _super.call(this, values) || this;
        }
        return Time;
    }(ROSLIB.Message));
    StdMsgs.Time = Time;
    var UInt16 = (function (_super) {
        __extends(UInt16, _super);
        function UInt16() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return UInt16;
    }(Byte));
    StdMsgs.UInt16 = UInt16;
    var UInt16MultiArray = (function (_super) {
        __extends(UInt16MultiArray, _super);
        function UInt16MultiArray() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return UInt16MultiArray;
    }(ByteMultiArray));
    StdMsgs.UInt16MultiArray = UInt16MultiArray;
    var UInt32 = (function (_super) {
        __extends(UInt32, _super);
        function UInt32() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return UInt32;
    }(Byte));
    StdMsgs.UInt32 = UInt32;
    var UInt32MultiArray = (function (_super) {
        __extends(UInt32MultiArray, _super);
        function UInt32MultiArray() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return UInt32MultiArray;
    }(ByteMultiArray));
    StdMsgs.UInt32MultiArray = UInt32MultiArray;
    var UInt64 = (function (_super) {
        __extends(UInt64, _super);
        function UInt64() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return UInt64;
    }(Byte));
    StdMsgs.UInt64 = UInt64;
    var UInt64MultiArray = (function (_super) {
        __extends(UInt64MultiArray, _super);
        function UInt64MultiArray() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return UInt64MultiArray;
    }(ByteMultiArray));
    StdMsgs.UInt64MultiArray = UInt64MultiArray;
    var UInt8 = (function (_super) {
        __extends(UInt8, _super);
        function UInt8() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return UInt8;
    }(Byte));
    StdMsgs.UInt8 = UInt8;
    var UInt8MultiArray = (function (_super) {
        __extends(UInt8MultiArray, _super);
        function UInt8MultiArray() {
            return _super !== null && _super.apply(this, arguments) || this;
        }
        return UInt8MultiArray;
    }(ByteMultiArray));
    StdMsgs.UInt8MultiArray = UInt8MultiArray;
})(StdMsgs || (StdMsgs = {}));
var StdSrvs;
(function (StdSrvs) {
    var EmptyRequest = (function (_super) {
        __extends(EmptyRequest, _super);
        function EmptyRequest(values) {
            return _super.call(this, values) || this;
        }
        return EmptyRequest;
    }(ROSLIB.ServiceRequest));
    StdSrvs.EmptyRequest = EmptyRequest;
    var EmptyResponse = (function (_super) {
        __extends(EmptyResponse, _super);
        function EmptyResponse(values) {
            return _super.call(this, values) || this;
        }
        return EmptyResponse;
    }(ROSLIB.ServiceResponse));
    StdSrvs.EmptyResponse = EmptyResponse;
    var SetBoolRequest = (function (_super) {
        __extends(SetBoolRequest, _super);
        function SetBoolRequest(values) {
            return _super.call(this, values) || this;
        }
        return SetBoolRequest;
    }(ROSLIB.ServiceRequest));
    StdSrvs.SetBoolRequest = SetBoolRequest;
    var SetBoolResponse = (function (_super) {
        __extends(SetBoolResponse, _super);
        function SetBoolResponse(values) {
            return _super.call(this, values) || this;
        }
        return SetBoolResponse;
    }(ROSLIB.ServiceResponse));
    StdSrvs.SetBoolResponse = SetBoolResponse;
    var TriggerRequest = (function (_super) {
        __extends(TriggerRequest, _super);
        function TriggerRequest(values) {
            return _super.call(this, values) || this;
        }
        return TriggerRequest;
    }(ROSLIB.ServiceRequest));
    StdSrvs.TriggerRequest = TriggerRequest;
    var TriggerResponse = (function (_super) {
        __extends(TriggerResponse, _super);
        function TriggerResponse(values) {
            return _super.call(this, values) || this;
        }
        return TriggerResponse;
    }(ROSLIB.ServiceResponse));
    StdSrvs.TriggerResponse = TriggerResponse;
})(StdSrvs || (StdSrvs = {}));
//# sourceMappingURL=rosmsgs.js.map