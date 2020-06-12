function init() {
  let hatebVizConfig: HATEBVizConfig = {
    rosBridgeUrl: "ws://localhost:9090",
    resourcesPath: "http://resources.robotwebtools.org/",
    viewerDivId: "viewer",
    fixedFrame: "map",
    mapOpacity: 0.7,
    globalPathTopic: "/move_base_node/GlobalPlanner/plan",
    localPathTopic: "/move_base_node/TebLocalPlannerROS/local_plan",
    localTrajTopic: "/move_base_node/TebLocalPlannerROS/local_traj",
    robotLocalPlanPosesTopic: "/move_base_node/TebLocalPlannerROS/local_plan_poses",
    robotLocalPlanFpPosesTopic: "/move_base_node/TebLocalPlannerROS/local_plan_fp_poses",
    humanGlobalPlansTopic: "/move_base_node/TebLocalPlannerROS/human_global_plans",
    humanLocalPlansTopic: "/move_base_node/TebLocalPlannerROS/human_local_plans",
    humanLocalPlanPosesTopic: "/move_base_node/TebLocalPlannerROS/human_local_plan_poses",
    humanLocalPlanFpPosesTopic: "/move_base_node/TebLocalPlannerROS/human_local_plan_fp_poses",
    tebMarkersTopic: "/move_base_node/TebLocalPlannerROS/teb_markers",
    robotGoalTopic: "/move_base_simple/goal",
    robotVelocityTopic: "/cmd_vel",
    moveBaseActionServerName: "/move_base",
    moveBaseActionName: "/move_base_msgs/MoveBaseAction",
    localCostmapTopic: "/move_base_node/local_costmap/costmap",
    teleportRobotTopic: "/pr2_teleport_pose",
    setRobotPoseTopic: "/initialpose",
    getPlanService: "/move_base_node/GlobalPlanner/make_plan",
    optimizePlansService: "/move_base_node/TebLocalPlannerROS/optimize",
    humanMarkersTopic: "/optitrack_person/tracked_persons_markers",
    humanPredictionMarkerTopic: "/human_pose_prediction/predicted_human_poses",
    publishHumanPredictionMarkersSrvName: "/human_pose_prediction/publish_prediction_markers",
    simHumanPlannerPathsTopic: "/move_humans_node/MultiGoalPlanner/plans",
    simHumanPlannerPosesTopic: "/move_humans_node/MultiGoalPlanner/plans_poses",
    simHumanControllerPathsTopic: "/move_humans_node/TeleportController/plans",
    simHumanMarkersTopic: "/move_humans_node/human_markers",
    simDynamicReconfigureService: "/move_humans_node/set_parameters",
    simPlannerDynamicReconfigureService: "/move_humans_node/MultiGoalPlanner/set_parameters",
    simControllerDynamicReconfigureService: "/move_humans_node/TeleportController/set_parameters",
    resetSimulationService: "/move_humans_node/reset_simulation",
    addHumanService: "/move_humans_node/add_human",
    deleteHumanService: "/move_humans_node/delete_human",
    addSubgoalService: "/move_humans_node/add_sub_goal",
    updateGoalService: "/move_humans_node/update_goal",
    teleportHumanService: "/move_humans_node/teleport_human",
    followExternalPathsService: "/move_humans_node/follow_external_paths",
    dynamicReconfigureService: "/move_base_node/TebLocalPlannerROS/set_parameters",
    experimentsFile: "experiments.json",
  };

  let hatebViz = new HATEBViz(hatebVizConfig);
}

interface HATEBVizConfig {
  rosBridgeUrl: string;
  resourcesPath: string;
  viewerDivId: string;
  fixedFrame: string;
  mapOpacity: number;
  globalPathTopic: string;
  localPathTopic: string;
  localTrajTopic: string;
  robotLocalPlanPosesTopic: string;
  robotLocalPlanFpPosesTopic: string;
  humanGlobalPlansTopic: string;
  humanLocalPlansTopic: string;
  humanLocalPlanPosesTopic: string;
  humanLocalPlanFpPosesTopic: string;
  tebMarkersTopic: string;
  robotGoalTopic: string;
  robotVelocityTopic: string;
  moveBaseActionServerName: string;
  moveBaseActionName: string;
  localCostmapTopic: string;
  teleportRobotTopic: string;
  setRobotPoseTopic: string;
  getPlanService: string;
  optimizePlansService: string;
  humanMarkersTopic: string;
  humanPredictionMarkerTopic: string;
  publishHumanPredictionMarkersSrvName: string;
  simHumanPlannerPathsTopic: string;
  simHumanPlannerPosesTopic: string;
  simHumanControllerPathsTopic: string;
  simHumanMarkersTopic: string;
  simDynamicReconfigureService: string;
  simPlannerDynamicReconfigureService: string;
  simControllerDynamicReconfigureService: string;
  resetSimulationService: string;
  addHumanService: string;
  deleteHumanService: string;
  addSubgoalService: string;
  updateGoalService: string;
  teleportHumanService: string;
  followExternalPathsService: string;
  dynamicReconfigureService: string;
  experimentsFile: string;
}

class OptimizeRequest extends ROSLIB.ServiceRequest {
  public robot_plan: NavMsgs.Path;
  public human_path_array: HANPMsgs.HumanPathArray;

  constructor(values: {
    robot_plan: NavMsgs.Path;
    human_path_array: HANPMsgs.HumanPathArray;
  }) { super(values); }
}

class OptimizeResponse extends ROSLIB.ServiceResponse {
  public success: boolean;
  public message: string;

  constructor(values: {
    success: boolean;
    message: string;
  }) { super(values); }
}

interface Experiment {
  name: string;
  robot_start: GeometryMsgs.Pose;
  robot_goal: GeometryMsgs.Pose;
  human_id: number;
  human_start: GeometryMsgs.Pose;
  human_goal: GeometryMsgs.Pose;
  human_delay: number;
}

class HATEBViz {
  private config: HATEBVizConfig;
  private ros: ROSLIB.Ros;
  private viewer: ROS3DNAV.Viewer;
  private viewerDivId: string;
  private tfClient: ROSLIB.TFClient;
  private gridClient: ROS3DNAV.OccupancyGridClient;
  private localGridClient: ROS3D.OccupancyGridClient;
  private urdfClient: ROS3D.UrdfClient;

  private fixedFrame: string;
  private robotGoalTopic: ROSLIB.Topic;
  private robotVelocityTopic: ROSLIB.Topic;
  private moveBaseActionClient: ROSLIB.ActionClient;
  private teleportRobotTopic: ROSLIB.Topic;
  private setRobotPoseTopic: ROSLIB.Topic;
  private humanPredictionMarkers: ROS3D.MarkerArrayClient;
  private publishHumanPredictionMarkersClient: ROSLIB.Service;
  private dynamicReconfigureClient: ROSLIB.Service;

  private moveHumansViz: MoveHumans.MoveHumansViz;

  private experimets: Experiment[];

  private getPlanClient: ROSLIB.Service;
  private optimizePlansClient: ROSLIB.Service;
  private robotPlanRequest: NavMsgs.GetPlanRequest;
  private humanPlanRequest: NavMsgs.GetPlanRequest;
  private optimizeRequest: OptimizeRequest;

  private isSim: boolean;

  constructor(config: HATEBVizConfig) {

    // save configuration
    this.config = config;

    // setup ros
    this.ros = new ROSLIB.Ros();
    this.ros.on("error", this.rosConnectionError);
    this.ros.on("close", this.rosConnectionClosed);
    this.ros.on("connection", this.rosConnectionCreated);

    // connect to ros
    this.ros.connect(this.config.rosBridgeUrl);
  }

  private rosConnectionError = () => {
    document.getElementById("status").innerHTML = "Error connecting to rosbridge at "
      + this.config.rosBridgeUrl.toString() + ".";
  }
  private rosConnectionClosed = () => {
    document.getElementById("status").innerHTML = "Connetion to rosbridge at "
      + this.config.rosBridgeUrl.toString() + " is closed.";
  }

  private rosConnectionCreated = () => {
    document.getElementById("status").style.display = "none";
    // create the main viewer
    this.viewerDivId = this.config.viewerDivId;

    let storedCameraPose = { x: 20, y: 10, z: 25 };
    let storedCameraPoseString = localStorage.getItem("cameraPose");
    if (storedCameraPoseString != null) {
      storedCameraPose = JSON.parse(storedCameraPoseString);
    }

    let storedOrbitCenter = new THREE.Vector3();
    let storedOrbitCenterString = localStorage.getItem("orbitCenter");
    if (storedOrbitCenterString != null) {
      let storedOrbitCenterJSON = JSON.parse(storedOrbitCenterString);
      storedOrbitCenter.x = storedOrbitCenterJSON.x;
      storedOrbitCenter.y = storedOrbitCenterJSON.y;
      storedOrbitCenter.z = storedOrbitCenterJSON.z;
    }

    this.viewer = new ROS3DNAV.Viewer({
      divID: this.viewerDivId,
      width: document.getElementById(this.viewerDivId).clientWidth - 20,
      height: window.innerHeight - 18,
      antialias: true,
      cameraPose: storedCameraPose,
      center: storedOrbitCenter,
    });
    window.addEventListener("resize", this.resizeViewer, false);

    // setup a clietn ot litsen to TFs
    this.tfClient = new ROSLIB.TFClient({
      ros: this.ros,
      fixedFrame: this.config.fixedFrame,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 30.0,
    });
    this.fixedFrame = this.config.fixedFrame;

    // show the map
    this.gridClient = new ROS3DNAV.OccupancyGridClient({
      ros: this.ros,
      viewer: this.viewer,
      continuous: false,
      rootObject: this.viewer.scene,
      opacity: this.config.mapOpacity,
      tfClient: this.tfClient,
      // offsetPose: new ROSLIB.Pose({
      //   position: new ROSLIB.Vector3({ x: 0.0, y: 0.0, z: 0.0 }),
      //   orientation: new ROSLIB.Quaternion({ x: 0.0, y: 0.0, z: 0.0, w: 1.0 }),
      // }),
    });

    // // show local costmap
    // this.localGridClient = new ROS3D.OccupancyGridClient({
    //   ros: this.ros,
    //   continuous: true,
    //   tfClient: this.tfClient,
    //   rootObject: this.viewer.scene,
    //   opacity: Math.max(this.config.mapOpacity + 0.2, 1.0),
    //   topic: this.config.localCostmapTopic,
    //   offsetPose: new ROSLIB.Pose({
    //     position: new ROSLIB.Vector3({ x: 0.0, y: 0.0, z: -0.05 }),
    //     orientation: new ROSLIB.Quaternion({ x: 0.0, y: 0.0, z: 0.0, w: 1.0 }),
    //   }),
    // });

    // setup the urdf client
    this.urdfClient = new ROS3D.UrdfClient({
      ros: this.ros,
      tfClient: this.tfClient,
      path: this.config.resourcesPath,
      rootObject: this.viewer.scene,
      loader: ROS3D.COLLADA_LOADER_2,
    });

    // set up laser scanner client
    // let laserScan = new ROS3D.LaserScan({
    //   ros: this.ros,
    //   topic: "/scan",
    //   tfClient: this.tfClient,
    //   rootObject: this.viewer.scene,
    //   size: 0.2,
    // });

    // setup robot path clients
    let globalNavPath = new ROS3DNAV.Path({
      ros: this.ros,
      topic: this.config.globalPathTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
      color: 0x00ff00,
      zOffset: 0.02,
      width: 2,
    });
    // let localNavPath = new ROS3DNAV.Path({
    //   ros: this.ros,
    //   topic: this.config.localPathTopic,
    //   tfClient: this.tfClient,
    //   rootObject: this.viewer.scene,
    //   color: 0xff0000,
    //   zOffset: 0.04,
    //   width: 3,
    // });
    let localNavTraj = new ROS3DNAV.Trajectory({
      ros: this.ros,
      topic: this.config.localTrajTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
      color: 0xff0000,
      width: 3
    })
    let robotLocalPlanPoses = new ROS3D.PoseArray({
      ros: this.ros,
      topic: this.config.robotLocalPlanPosesTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
    });
    let robotLocalPlanFpPoses = new ROS3D.MarkerArrayClient({
      ros: this.ros,
      topic: this.config.robotLocalPlanFpPosesTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
    });

    // setup human marker client
    let humansMarkers = new ROS3D.MarkerArrayClient({
      ros: this.ros,
      topic: this.config.humanMarkersTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
    });

    // setup human paths clients
    let humanGlobalPlans = new ROS3DNAV.HumanPathArray({
      ros: this.ros,
      topic: this.config.humanGlobalPlansTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
      color: 0xFFD700,
      width: 3,
    });
    let humanLocalPlans = new ROS3DNAV.HumanTrajectoryArray({
      ros: this.ros,
      topic: this.config.humanLocalPlansTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
      color: 0x000080,
      width: 3,
    });
    let humanLocalPlanPoses = new ROS3D.PoseArray({
      ros: this.ros,
      topic: this.config.humanLocalPlanPosesTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
    });
    let humanLocalPlanFpPoses = new ROS3D.MarkerArrayClient({
      ros: this.ros,
      topic: this.config.humanLocalPlanFpPosesTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
    });

    // setup teb markers
    // let tebMarkers = new ROS3D.MarkerClient({
    //   ros: this.ros,
    //   topic: this.config.tebMarkersTopic,
    //   tfClient: this.tfClient,
    //   rootObject: this.viewer.scene,
    // });

    // setup human prediction markers
    let predictionMarkersCB = <HTMLInputElement>document.getElementById("prediction-markers-cb");
    if (predictionMarkersCB) {
      this.humanPredictionMarkers = new ROS3D.MarkerArrayClient({
        ros: this.ros,
        topic: this.config.humanPredictionMarkerTopic,
        tfClient: this.tfClient,
        rootObject: this.viewer.scene,
      });
      this.publishHumanPredictionMarkersClient = new ROSLIB.Service({
        ros: this.ros,
        name: this.config.publishHumanPredictionMarkersSrvName,
        serviceType: "std_srvs/SetBool",
      });
      predictionMarkersCB.onclick = this.showHumanPredictionsClicked;
    }

    this.ros.getNodes((nodeNames) => {
      for (let nodeName of nodeNames) {
        if (nodeName === "/move_humans_node") {
          let moveHumansConfig: MoveHumans.MoveHumansConfig = {
            ros: this.ros,
            viewer: this.viewer,
            viewerDivId: this.viewerDivId,
            tfClient: this.tfClient,
            gridClient: this.gridClient,
            fixedFrame: this.fixedFrame,
            humanPlannerPathsTopic: this.config.simHumanPlannerPathsTopic,
            humanPlannerPosesTopic: this.config.simHumanPlannerPosesTopic,
            humanControllerPathsTopic: this.config.simHumanControllerPathsTopic,
            humanMarkersTopic: this.config.simHumanMarkersTopic,
            resetSimulationService: this.config.resetSimulationService,
            addHumanService: this.config.addHumanService,
            deleteHumanService: this.config.deleteHumanService,
            addSubgoalService: this.config.addSubgoalService,
            updateGoalService: this.config.updateGoalService,
            teleportHumanService: this.config.teleportHumanService,
            followExternalPathsService: this.config.followExternalPathsService,
            dynamicReconfigureService: this.config.simDynamicReconfigureService,
            plannerDynamicReconfigureService: this.config.simPlannerDynamicReconfigureService,
            controllerDynamicReconfigureService: this.config.simControllerDynamicReconfigureService,
          };
          this.moveHumansViz = new MoveHumans.MoveHumansViz(moveHumansConfig);
        }

        if (nodeName === "/morse") {
          this.isSim = true;
        }
      }
    });

    let robotGoalBT = document.getElementById("robot-goal-bt");
    if (robotGoalBT) {
      this.robotGoalTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: this.config.robotGoalTopic,
        messageType: "geometry_msgs/PoseStamped",
      });
      robotGoalBT.onclick = this.setRobotGoalClicked;
    }

    this.robotVelocityTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: this.config.robotVelocityTopic,
      messageType: "geometry_msgs/Twist",
    });

    // setup move_base action clinet
    this.moveBaseActionClient = new ROSLIB.ActionClient({
      ros: this.ros,
      serverName: this.config.moveBaseActionServerName,
      actionName: this.config.moveBaseActionName,
      timeout: 5.0,
    });

    // setup teleportation of the robot
    let teleportRobotBT = document.getElementById("teleport-robot-bt");
    if (teleportRobotBT) {
      teleportRobotBT.onclick = this.teleportRobotClicked;
    }

    this.teleportRobotTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: this.config.teleportRobotTopic,
      messageType: "geometry_msgs/Pose",
    });
    this.setRobotPoseTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: this.config.setRobotPoseTopic,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    // setup dynamic reconfigure
    this.dynamicReconfigureClient = new ROSLIB.Service({
      ros: this.ros,
      name: this.config.dynamicReconfigureService,
      serviceType: "dynamic_reconfigure/Reconfigure",
    });
    (<HTMLInputElement>document.getElementById("robot-global-plan-cb")).onchange = this.visualizationOptionsChanged;
    (<HTMLInputElement>document.getElementById("robot-local-plan-cb")).onchange = this.visualizationOptionsChanged;
    (<HTMLInputElement>document.getElementById("robot-local-poses-cb")).onchange = this.visualizationOptionsChanged;
    (<HTMLInputElement>document.getElementById("human-global-plans-cb")).onchange = this.visualizationOptionsChanged;
    (<HTMLInputElement>document.getElementById("human-local-plans-cb")).onchange = this.visualizationOptionsChanged;
    (<HTMLInputElement>document.getElementById("human-local-poses-cb")).onchange = this.visualizationOptionsChanged;
    (<HTMLInputElement>document.getElementById("double-param-name-sl")).onchange = this.parameterChanged;
    (<HTMLInputElement>document.getElementById("double-param-value-ip"))
      .addEventListener("keyup", this.doubleParameterSet);
    (<HTMLInputElement>document.getElementById("bool-param-name-sl")).onchange = this.parameterChanged;
    (<HTMLInputElement>document.getElementById("bool-param-value-cb")).onchange = this.boolParameterSet;

    let req = new DynamicReconfigure.ReconfigureRequest({
      config: new DynamicReconfigure.Config({}),
    });
    this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureInitialResponse, this.failedServiceCallback);

    // setup testing
    let TRobotStartBT = document.getElementById("trobot-start-bt");
    let TRobotGoalBT = document.getElementById("trobot-goal-bt");

    if (TRobotStartBT && TRobotGoalBT) {
      this.getPlanClient = new ROSLIB.Service({
        ros: this.ros,
        name: this.config.getPlanService,
        serviceType: "nav_msgs/GetPlan",
      });

      this.optimizePlansClient = new ROSLIB.Service({
        ros: this.ros,
        name: this.config.optimizePlansService,
        serviceType: "teb_local_planner/Optimize",
      });

      TRobotStartBT.onclick = this.setTRobotStartClicked;
      TRobotGoalBT.onclick = this.setTRobotGoalClicked;
    }

    let THumanIDIP = document.getElementById("thuman-id-ip");
    let addTHumanBT = document.getElementById("add-thuman-bt");
    let deleteTHumanBT = document.getElementById("delete-thuman-bt");
    let goalTHumanBT = document.getElementById("goal-thuman-bt");

    if (THumanIDIP && addTHumanBT && deleteTHumanBT && goalTHumanBT) {
      THumanIDIP.onkeyup = this.THumanIDChanged;
      addTHumanBT.onclick = this.setTHumanStartClicked;
      deleteTHumanBT.onclick = this.setTHumanDeleteClicked;
      goalTHumanBT.onclick = this.setTHumanGoalClicked;
    }

    this.robotPlanRequest = new NavMsgs.GetPlanRequest({
      start: null,
      goal: null,
      tolerance: 0.1,
    });

    this.humanPlanRequest = new NavMsgs.GetPlanRequest({
      start: null,
      goal: null,
      tolerance: 0.1,
    });

    this.optimizeRequest = new OptimizeRequest({
      robot_plan: null,
      human_path_array: new HANPMsgs.HumanPathArray({
        header: new StdMsgs.Header({
          seq: 1,
          stamp: ROS3DNAV.Time.now(),
          frame_id: this.fixedFrame,
        }),
        paths: [],
      }),
    });

    let optimizeBT = document.getElementById("optimize-bt");
    if (optimizeBT) {
      optimizeBT.onclick = this.optimizeClicked;
    }

    // setup experiments
    let experimentSL = <HTMLInputElement>document.getElementById("experiments-sl");
    let experimentResetBT = document.getElementById("experiment-reset-bt");
    let experimentStartBT = document.getElementById("experiment-start-bt");

    if (experimentSL && experimentResetBT && experimentStartBT) {
      let request = new XMLHttpRequest();
      request.onload = () => {
        this.experimets = JSON.parse(request.responseText).experiments;
        this.populateExperiments();
      };
      request.open("get", this.config.experimentsFile, true);
      request.send();

      experimentSL.onchange = this.experimentChanged;
      experimentResetBT.onclick = this.experimentChanged;
      experimentStartBT.onclick = this.startExperimentClicked;
    }

    // store camera position and orbit control every second
    setInterval(() => {
      localStorage.setItem("cameraPose", JSON.stringify(this.viewer.camera.position));
      localStorage.setItem("orbitCenter", JSON.stringify(this.viewer.cameraControls.center));
    }, 1000);
  }

  private resizeViewer = () => {
    this.viewer.resize(document.getElementById(this.viewerDivId).clientWidth - 20,
      window.innerHeight - 18);
  }

  private failedServiceCallback = (message: string) => {
    this.setMessage("Service call failed:\n" + message);
  }

  private showHumanPredictionsClicked = () => {
    let req = new StdSrvs.SetBoolRequest({
      data: (<HTMLInputElement>document.getElementById("prediction-markers-cb")).checked,
    });
    this.publishHumanPredictionMarkersClient.callService(req,
      this.publishHumanPredictionMarkersResponse, this.failedServiceCallback);
  }

  private publishHumanPredictionMarkersResponse = (res: StdSrvs.SetBoolResponse) => {
    if (!res.success) {
      this.setMessage("Setting human prediction markers service failed:\n" + res.message);
    } else {
      this.setMessage(res.message);
    }
  }

  private startArrowControl(poseReceiveCB: any) {
    if (!this.gridClient.createArrowControl(this.arrowCallback, { poseReceiveCB: poseReceiveCB })) {
      this.setMessage("Please wait, map is being downloaded");
    } else {
      this.setMessage("Please choose a pose on the map");
    }
  }

  private arrowCallback = (success: boolean, position?: THREE.Vector3, orientation?: THREE.Quaternion,
    message?: string, senderInfo?: any) => {
    if (success) {
      senderInfo.poseReceiveCB(position, orientation);
    } else {
      this.setMessage(message);
    }
  }

  private setRobotGoalClicked = () => {
    this.startArrowControl(this.publishRobotGoal);
  }

  private teleportRobotClicked = () => {
    this.startArrowControl(this.teleportRobot);
  }

  private publishRobotGoal = (pose: THREE.Vector3, direction: THREE.Quaternion) => {
    this.stopRobot();

    let robotGoal = new GeometryMsgs.PoseStamped({
      header: new StdMsgs.Header({
        seq: 1,
        stamp: ROS3DNAV.Time.now(),
        frame_id: this.fixedFrame,
      }),
      pose: new GeometryMsgs.Pose({
        position: new GeometryMsgs.Point({
          x: pose.x,
          y: pose.y,
          z: pose.z,
        }),
        orientation: new GeometryMsgs.Quaternion({
          x: direction.x,
          y: direction.y,
          z: direction.z,
          w: direction.w,
        }),
      }),
    });
    // this.robotGoalTopic.publish(robotGoal);

    let goal = new ROSLIB.Goal({
      actionClient: this.moveBaseActionClient,
      goalMessage: new MoveBaseMsgs.MoveBaseGoal({
        target_pose: robotGoal,
      }),
    });
    goal.send();
  }

  private teleportRobot = (pose: THREE.Vector3, direction: THREE.Quaternion) => {
    this.stopRobot();

    let robotPose = new GeometryMsgs.Pose({
      position: new GeometryMsgs.Point({
        x: pose.x,
        y: pose.y,
        z: pose.z,
      }),
      orientation: new GeometryMsgs.Quaternion({
        x: direction.x,
        y: direction.y,
        z: direction.z,
        w: direction.w,
      }),
    });

    if (this.isSim) {
      this.teleportRobotZero();
      setTimeout(() => this.teleportRobotTopic.publish(robotPose), 500);
    } else {
      let robotPoseCov = new GeometryMsgs.PoseWithCovarianceStamped({
        header: new StdMsgs.Header({
          seq: 1,
          stamp: ROS3DNAV.Time.now(),
          frame_id: this.fixedFrame,
        }),
        pose: new GeometryMsgs.PoseWithCovariance({
          pose: robotPose,
          covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        }),
      });
      this.setRobotPoseTopic.publish(robotPoseCov);
    }
  }

  private stopRobot() {
    this.moveBaseActionClient.cancel();
    let zeroVelocity = new GeometryMsgs.Twist({
      linear: new GeometryMsgs.Vector3({
        x: 0.0,
        y: 0.0,
        z: 0.0,
      }),
      angular: new GeometryMsgs.Vector3({
        x: 0.0,
        y: 0.0,
        z: 0.0,
      }),
    });
    setTimeout(() => this.robotVelocityTopic.publish(zeroVelocity), 100);
  }

  private setMessage(message: string) {
    (<HTMLInputElement>document.getElementById("message-lb")).value = message;
  }

  private updateMessage(message: string) {
    (<HTMLInputElement>document.getElementById("message-lb")).value += message;
  }

  private visualizationOptionsChanged = () => {
    this.disablePlannerVisualizationOptions();
    let showRobotGlobalPlan = (<HTMLInputElement>document.getElementById("robot-global-plan-cb")).checked;
    let showRobotLocalPlan = (<HTMLInputElement>document.getElementById("robot-local-plan-cb")).checked;
    let showRobotLocalPlanPoses = (<HTMLInputElement>document.getElementById("robot-local-poses-cb")).checked;
    let showHumanGlobalPlans = (<HTMLInputElement>document.getElementById("human-global-plans-cb")).checked;
    let showHumanLocalPlans = (<HTMLInputElement>document.getElementById("human-local-plans-cb")).checked;
    let showHumanLocalPlanPoses = (<HTMLInputElement>document.getElementById("human-local-poses-cb")).checked;
    let req = new DynamicReconfigure.ReconfigureRequest({
      config: new DynamicReconfigure.Config({
        bools: [
          new DynamicReconfigure.BoolParameter({
            name: "publish_robot_global_plan",
            value: showRobotGlobalPlan,
          }),
          new DynamicReconfigure.BoolParameter({
            name: "publish_robot_local_plan",
            value: showRobotLocalPlan,
          }),
          new DynamicReconfigure.BoolParameter({
            name: "publish_robot_local_plan_poses",
            value: showRobotLocalPlanPoses,
          }),
          new DynamicReconfigure.BoolParameter({
            name: "publish_human_global_plans",
            value: showHumanGlobalPlans,
          }),
          new DynamicReconfigure.BoolParameter({
            name: "publish_human_local_plans",
            value: showHumanLocalPlans,
          }),
          new DynamicReconfigure.BoolParameter({
            name: "publish_human_local_plan_poses",
            value: showHumanLocalPlanPoses,
          }),
        ],
      }),
    });
    this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureResponse, this.failedServiceCallback);
  }

  private dynamicReconfigureInitialResponse = (res: DynamicReconfigure.ReconfigureResponse) => {
    let storedBoolName = JSON.parse(localStorage.getItem("selectedBool"));
    let boolSelectionList = (<HTMLSelectElement>document.getElementById("bool-param-name-sl"));
    for (let boolParam of res.config.bools) {
      let boolOption = document.createElement("option");
      boolOption.text = boolParam.name;
      boolSelectionList.add(boolOption);
      if (boolOption.text === storedBoolName) {
        boolOption.selected = true;
      }
    }

    let storedDoubleName = JSON.parse(localStorage.getItem("selectedDouble"));
    let doubleSelectionList = (<HTMLSelectElement>document.getElementById("double-param-name-sl"));
    for (let doubleParam of res.config.doubles) {
      let doubleOption = document.createElement("option");
      doubleOption.text = doubleParam.name;
      doubleSelectionList.add(doubleOption);
      if (doubleOption.text === storedDoubleName) {
        doubleOption.selected = true;
      }
    }

  }

  private dynamicReconfigureResponse = (res: DynamicReconfigure.ReconfigureResponse) => {
    let boolSelectionList = (<HTMLSelectElement>document.getElementById("bool-param-name-sl"));
    let boolParamName = boolSelectionList.options[boolSelectionList.selectedIndex].innerHTML;
    let boolParamFound = false;

    for (let boolParam of res.config.bools) {
      if (boolParam.name === "publish_robot_global_plan") {
        (<HTMLInputElement>document.getElementById("robot-global-plan-cb")).checked = boolParam.value;
      }
      if (boolParam.name === "publish_robot_local_plan") {
        (<HTMLInputElement>document.getElementById("robot-local-plan-cb")).checked = boolParam.value;
      }
      if (boolParam.name === "publish_robot_local_plan_poses") {
        (<HTMLInputElement>document.getElementById("robot-local-poses-cb")).checked = boolParam.value;
      }
      if (boolParam.name === "publish_human_global_plans") {
        (<HTMLInputElement>document.getElementById("human-global-plans-cb")).checked = boolParam.value;
      }
      if (boolParam.name === "publish_human_local_plans") {
        (<HTMLInputElement>document.getElementById("human-local-plans-cb")).checked = boolParam.value;
      }
      if (boolParam.name === "publish_human_local_plan_poses") {
        (<HTMLInputElement>document.getElementById("human-local-poses-cb")).checked = boolParam.value;
      }
      if (boolParam.name === boolParamName) {
        (<HTMLInputElement>document.getElementById("bool-param-value-cb")).checked = boolParam.value;
        boolParamFound = true;
      }
    }
    this.enablePlannerVisualizationOptions();

    if (!boolParamFound) {
      (<HTMLInputElement>document.getElementById("bool-param-value-cb")).checked = false;
      // this.setMessage("No param named " + boolParamName + " found");
    }

    let doubleSelectionList = (<HTMLSelectElement>document.getElementById("double-param-name-sl"));
    let doubleParamName = doubleSelectionList.options[doubleSelectionList.selectedIndex].innerHTML;
    let doubleParamFound = false;
    for (let doubleParam of res.config.doubles) {
      if (doubleParam.name === doubleParamName) {
        (<HTMLInputElement>document.getElementById("double-param-value-ip")).value = doubleParam.value.toString();
        doubleParamFound = true;
      }
    }
    if (!doubleParamFound) {
      (<HTMLInputElement>document.getElementById("double-param-value-ip")).value = "NA";
      // this.setMessage("No param named " + doubleParamName + " found");
    }
  }

  private disablePlannerVisualizationOptions = () => {
    (<HTMLInputElement>document.getElementById("robot-global-plan-cb")).disabled = true;
    (<HTMLInputElement>document.getElementById("robot-local-plan-cb")).disabled = true;
    (<HTMLInputElement>document.getElementById("robot-local-poses-cb")).disabled = true;
    (<HTMLInputElement>document.getElementById("human-global-plans-cb")).disabled = true;
    (<HTMLInputElement>document.getElementById("human-local-plans-cb")).disabled = true;
    (<HTMLInputElement>document.getElementById("human-local-poses-cb")).disabled = true;
  }

  private enablePlannerVisualizationOptions = () => {
    (<HTMLInputElement>document.getElementById("robot-global-plan-cb")).disabled = false;
    (<HTMLInputElement>document.getElementById("robot-local-plan-cb")).disabled = false;
    (<HTMLInputElement>document.getElementById("robot-local-poses-cb")).disabled = false;
    (<HTMLInputElement>document.getElementById("human-global-plans-cb")).disabled = false;
    (<HTMLInputElement>document.getElementById("human-local-plans-cb")).disabled = false;
    (<HTMLInputElement>document.getElementById("human-local-poses-cb")).disabled = false;
  }

  private parameterChanged = () => {
    let boolSelectionList = (<HTMLSelectElement>document.getElementById("bool-param-name-sl"));
    let boolParamName = boolSelectionList.options[boolSelectionList.selectedIndex].innerHTML;
    localStorage.setItem("selectedBool", JSON.stringify(boolParamName));

    let doubleSelectionList = (<HTMLSelectElement>document.getElementById("double-param-name-sl"));
    let doubleParamName = doubleSelectionList.options[doubleSelectionList.selectedIndex].innerHTML;
    localStorage.setItem("selectedDouble", JSON.stringify(doubleParamName));

    let req = new DynamicReconfigure.ReconfigureRequest({
      config: new DynamicReconfigure.Config({}),
    });
    this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureResponse, this.failedServiceCallback);
  }

  private doubleParameterSet = (event: KeyboardEvent) => {
    event.preventDefault();
    if (event.keyCode !== 13) { // Enter pressed
      return;
    }

    let selectionList = (<HTMLSelectElement>document.getElementById("double-param-name-sl"));
    let paramName = selectionList.options[selectionList.selectedIndex].innerHTML;
    let paramInput = (<HTMLInputElement>document.getElementById("double-param-value-ip")).value;
    if (paramInput.length > 0) {
      let paramValue = Number(paramInput);
      if (!isNaN(paramValue)) {
        let req = new DynamicReconfigure.ReconfigureRequest({
          config: new DynamicReconfigure.Config({
            doubles: [
              new DynamicReconfigure.DoubleParameter({
                name: paramName,
                value: paramValue,
              }),
            ],
          }),
        });
        this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureResponse, this.failedServiceCallback);
        this.setMessage("Setting " + paramName + " to " + paramValue.toString());
      } else {
        this.setMessage("Param value is NaN");
      }
    } else {
      this.setMessage("Please provide a parameter value");
    }
  }

  private boolParameterSet = () => {
    let selectionList = (<HTMLSelectElement>document.getElementById("bool-param-name-sl"));
    let paramName = selectionList.options[selectionList.selectedIndex].innerHTML;
    let paramValue = (<HTMLInputElement>document.getElementById("bool-param-value-cb")).checked;
    let req = new DynamicReconfigure.ReconfigureRequest({
      config: new DynamicReconfigure.Config({
        bools: [
          new DynamicReconfigure.BoolParameter({
            name: paramName,
            value: paramValue,
          }),
        ],
      }),
    });
    this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureResponse, this.failedServiceCallback);
    this.setMessage("Setting " + paramName + " to " + paramValue);
  }

  private populateExperiments = () => {
    let experimentsList = (<HTMLSelectElement>document.getElementById("experiments-sl"));
    for (let experiment of this.experimets) {
      let option = document.createElement("option");
      option.text = experiment.name;
      experimentsList.add(option);
    }
  }

  private experimentChanged = () => {
    let experimentsList = (<HTMLSelectElement>document.getElementById("experiments-sl"));
    let selectedExperiment = experimentsList.options[experimentsList.selectedIndex].innerHTML;
    for (let experiment of this.experimets) {
      if (experiment.name === selectedExperiment) {
        this.stopRobot();
        this.teleportRobotZero();
        setTimeout(() => this.teleportRobotTopic.publish(experiment.robot_start), 500);
        this.moveHumansViz.teleportHuman(experiment.human_id, experiment.human_start);
        this.setMessage("Initial positions set for " + selectedExperiment);
        break;
      }
    }
  }

  private startExperimentClicked = () => {
    let experimentsList = (<HTMLSelectElement>document.getElementById("experiments-sl"));
    let selectedExperiment = experimentsList.options[experimentsList.selectedIndex].innerHTML;
    for (let experiment of this.experimets) {
      if (experiment.name === selectedExperiment) {
        let goal = new ROSLIB.Goal({
          actionClient: this.moveBaseActionClient,
          goalMessage: new MoveBaseMsgs.MoveBaseGoal({
            target_pose: new GeometryMsgs.PoseStamped({
              header: new StdMsgs.Header({
                seq: 1,
                stamp: ROS3DNAV.Time.now(),
                frame_id: this.fixedFrame,
              }),
              pose: experiment.robot_goal,
            }),
          }),
        });
        goal.send();
        setTimeout(() => this.moveHumansViz.setHumanGoal(experiment.human_id, experiment.human_goal),
          experiment.human_delay * 1000);
        this.setMessage("Started " + selectedExperiment);
        break;
      }
    }
  }

  private teleportRobotZero() {
    let zeroPose = new GeometryMsgs.Pose({
      position: new GeometryMsgs.Point({
        x: 0.0,
        y: 0.0,
        z: 0.0,
      }),
      orientation: new GeometryMsgs.Quaternion({
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
      }),
    });
    this.teleportRobotTopic.publish(zeroPose);
  }

  private setTRobotStartClicked = () => {
    this.startArrowControl(this.setTRobotStart);
  }

  private setTRobotGoalClicked = () => {
    this.startArrowControl(this.setTRobotGoal);
  }

  private setTRobotStart = (pose: THREE.Vector3, direction: THREE.Quaternion) => {
    this.setTRobotStartGoal(pose, direction, true);
  }

  private setTRobotGoal = (pose: THREE.Vector3, direction: THREE.Quaternion) => {
    this.setTRobotStartGoal(pose, direction, false);
  }

  private setTRobotStartGoal = (pose: THREE.Vector3, direction: THREE.Quaternion, isStart: boolean) => {
    let selectedPose = new GeometryMsgs.PoseStamped({
      header: new StdMsgs.Header({
        seq: 1,
        stamp: ROS3DNAV.Time.now(),
        frame_id: this.fixedFrame,
      }),
      pose: new GeometryMsgs.Pose({
        position: new GeometryMsgs.Point({
          x: pose.x,
          y: pose.y,
          z: pose.z,
        }),
        orientation: new GeometryMsgs.Quaternion({
          x: direction.x,
          y: direction.y,
          z: direction.z,
          w: direction.w,
        }),
      }),
    });

    if (isStart) {
      this.robotPlanRequest.start = selectedPose;
      this.teleportRobot(pose, direction);
    } else {
      this.robotPlanRequest.goal = selectedPose;
    }

    if (this.robotPlanRequest.start && this.robotPlanRequest.goal) {
      this.getPlanClient.callService(this.robotPlanRequest, this.getRobotPlanServiceResponse,
        this.failedServiceCallback);
    }
  }

  private setTHumanStartClicked = () => {
    this.startArrowControl(this.setTHumanStart);
  }

  private setTHumanGoalClicked = () => {
    this.startArrowControl(this.setTHumanGoal);
  }

  private setTHumanDeleteClicked = () => {
    let humanID = this.validateId("thuman-id-ip");
    if (humanID !== -1) {
      for (let i = this.optimizeRequest.human_path_array.paths.length - 1; i >= 0; i--) {
        if (this.optimizeRequest.human_path_array.paths[i].id === humanID) {
          this.optimizeRequest.human_path_array.paths.splice(i);
          this.setMessage("Deleted human " + humanID);
          break;
        }
      }
    }
    this.humanPlanRequest.goal = null;
    this.humanPlanRequest.start = null;
  }

  private THumanIDChanged = () => {
    this.humanPlanRequest.goal = null;
    this.humanPlanRequest.start = null;
  }

  private setTHumanStart = (pose: THREE.Vector3, direction: THREE.Quaternion) => {
    this.setTHumanStartGoal(pose, direction, true);
  }

  private setTHumanGoal = (pose: THREE.Vector3, direction: THREE.Quaternion) => {
    this.setTHumanStartGoal(pose, direction, false);
  }

  private setTHumanStartGoal = (pose: THREE.Vector3, direction: THREE.Quaternion, isStart: boolean) => {
    let humanID = this.validateId("thuman-id-ip");
    if (humanID !== -1) {
      let selectedPose = new GeometryMsgs.PoseStamped({
        header: new StdMsgs.Header({
          seq: 1,
          stamp: ROS3DNAV.Time.now(),
          frame_id: this.fixedFrame,
        }),
        pose: new GeometryMsgs.Pose({
          position: new GeometryMsgs.Point({
            x: pose.x,
            y: pose.y,
            z: pose.z,
          }),
          orientation: new GeometryMsgs.Quaternion({
            x: direction.x,
            y: direction.y,
            z: direction.z,
            w: direction.w,
          }),
        }),
      });

      if (isStart) {
        this.humanPlanRequest.start = selectedPose;
        this.moveHumansViz.teleportHuman(humanID, selectedPose.pose);
      } else {
        this.humanPlanRequest.goal = selectedPose;
      }

      if (this.humanPlanRequest.start && this.humanPlanRequest.goal) {
        this.getPlanClient.callService(this.humanPlanRequest, this.getHumanPlanServiceResponse,
          this.failedServiceCallback);
      }
    }
  }

  private validateId(elementID: string) {
    let input = (<HTMLInputElement>document.getElementById(elementID)).value;
    if (input.length > 0) {
      let value = Number(input);
      if (!isNaN(value) && value % 1 === 0 && value >= 0) {
        return value;
      }
    }
    this.setMessage("Invalid human id");
    return -1;
  }

  private getRobotPlanServiceResponse = (res: NavMsgs.GetPlanResponse) => {
    if (res.plan.poses.length > 0) {
      this.optimizeRequest.robot_plan = res.plan;
    }
    this.setMessage("got robot plan of size " + res.plan.poses.length);
  }

  private getHumanPlanServiceResponse = (res: NavMsgs.GetPlanResponse) => {
    let humanID = this.validateId("thuman-id-ip");
    if (humanID !== -1) {
      if (res.plan.poses.length > 0) {
        let humanPath = new HANPMsgs.HumanPath({
          header: new StdMsgs.Header({
            seq: 0,
            stamp: ROS3DNAV.Time.now(),
            frame_id: this.config.fixedFrame,
          }),
          id: humanID,
          path: res.plan,
        });

        for (let i = this.optimizeRequest.human_path_array.paths.length - 1; i >= 0; i--) {
          if (this.optimizeRequest.human_path_array.paths[i].id === humanID) {
            this.optimizeRequest.human_path_array.paths.splice(i);
            break;
          }
        }
        this.optimizeRequest.human_path_array.paths.push(humanPath);
      }
      this.setMessage("got human " + humanID + " plan of size " + res.plan.poses.length);
    }
  }

  private optimizeClicked = () => {
    if (this.optimizeRequest.robot_plan) {
      let humanPathsSize = this.optimizeRequest.human_path_array.paths.length;
      let humanPathsSizes = humanPathsSize === 1 ? "(size: " : "(sizes: ";
      for (let humanPath of this.optimizeRequest.human_path_array.paths) {
        humanPathsSizes += humanPath.path.poses.length + "'";
      }
      humanPathsSizes.slice(0, -1);
      humanPathsSizes += ")";
      this.setMessage("Optimizing " + humanPathsSize + " human " + (humanPathsSize === 1 ? "plan" : "plans") +
        " and robot plan of size " + this.optimizeRequest.robot_plan.poses.length);

      this.optimizePlansClient.callService(this.optimizeRequest, this.optimizeServiceResponse,
        this.failedServiceCallback);
    } else {
      this.setMessage("Please at least set a robot plan to optimize");
    }
  }

  private optimizeServiceResponse = (res: OptimizeResponse) => {
    this.setMessage("Optimizatin " + (res.success ? "succeded" : "failed") + "\n" + res.message);
  }
}
