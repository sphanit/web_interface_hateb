function init() {
    let hatebVizConfig = {
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
class OptimizeRequest extends ROSLIB.ServiceRequest {
    constructor(values) { super(values); }
}
class OptimizeResponse extends ROSLIB.ServiceResponse {
    constructor(values) { super(values); }
}
class HATEBViz {
    constructor(config) {
        this.rosConnectionError = () => {
            document.getElementById("status").innerHTML = "Error connecting to rosbridge at "
                + this.config.rosBridgeUrl.toString() + ".";
        };
        this.rosConnectionClosed = () => {
            document.getElementById("status").innerHTML = "Connetion to rosbridge at "
                + this.config.rosBridgeUrl.toString() + " is closed.";
        };
        this.rosConnectionCreated = () => {
            document.getElementById("status").style.display = "none";
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
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                fixedFrame: this.config.fixedFrame,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 30.0,
            });
            this.fixedFrame = this.config.fixedFrame;
            this.gridClient = new ROS3DNAV.OccupancyGridClient({
                ros: this.ros,
                viewer: this.viewer,
                continuous: false,
                rootObject: this.viewer.scene,
                opacity: this.config.mapOpacity,
                tfClient: this.tfClient,
            });
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                tfClient: this.tfClient,
                path: this.config.resourcesPath,
                rootObject: this.viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2,
            });
            let globalNavPath = new ROS3DNAV.Path({
                ros: this.ros,
                topic: this.config.globalPathTopic,
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
                color: 0x00ff00,
                zOffset: 0.02,
                width: 2,
            });
            let localNavTraj = new ROS3DNAV.Trajectory({
                ros: this.ros,
                topic: this.config.localTrajTopic,
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
                color: 0xff0000,
                width: 3
            });
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
            let humansMarkers = new ROS3D.MarkerArrayClient({
                ros: this.ros,
                topic: this.config.humanMarkersTopic,
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
            });
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
            let predictionMarkersCB = document.getElementById("prediction-markers-cb");
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
                        let moveHumansConfig = {
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
            this.moveBaseActionClient = new ROSLIB.ActionClient({
                ros: this.ros,
                serverName: this.config.moveBaseActionServerName,
                actionName: this.config.moveBaseActionName,
                timeout: 5.0,
            });
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
            this.dynamicReconfigureClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.dynamicReconfigureService,
                serviceType: "dynamic_reconfigure/Reconfigure",
            });
            document.getElementById("robot-global-plan-cb").onchange = this.visualizationOptionsChanged;
            document.getElementById("robot-local-plan-cb").onchange = this.visualizationOptionsChanged;
            document.getElementById("robot-local-poses-cb").onchange = this.visualizationOptionsChanged;
            document.getElementById("human-global-plans-cb").onchange = this.visualizationOptionsChanged;
            document.getElementById("human-local-plans-cb").onchange = this.visualizationOptionsChanged;
            document.getElementById("human-local-poses-cb").onchange = this.visualizationOptionsChanged;
            document.getElementById("double-param-name-sl").onchange = this.parameterChanged;
            document.getElementById("double-param-value-ip")
                .addEventListener("keyup", this.doubleParameterSet);
            document.getElementById("bool-param-name-sl").onchange = this.parameterChanged;
            document.getElementById("bool-param-value-cb").onchange = this.boolParameterSet;
            let req = new DynamicReconfigure.ReconfigureRequest({
                config: new DynamicReconfigure.Config({}),
            });
            this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureInitialResponse, this.failedServiceCallback);
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
            let experimentSL = document.getElementById("experiments-sl");
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
            setInterval(() => {
                localStorage.setItem("cameraPose", JSON.stringify(this.viewer.camera.position));
                localStorage.setItem("orbitCenter", JSON.stringify(this.viewer.cameraControls.center));
            }, 1000);
        };
        this.resizeViewer = () => {
            this.viewer.resize(document.getElementById(this.viewerDivId).clientWidth - 20, window.innerHeight - 18);
        };
        this.failedServiceCallback = (message) => {
            this.setMessage("Service call failed:\n" + message);
        };
        this.showHumanPredictionsClicked = () => {
            let req = new StdSrvs.SetBoolRequest({
                data: document.getElementById("prediction-markers-cb").checked,
            });
            this.publishHumanPredictionMarkersClient.callService(req, this.publishHumanPredictionMarkersResponse, this.failedServiceCallback);
        };
        this.publishHumanPredictionMarkersResponse = (res) => {
            if (!res.success) {
                this.setMessage("Setting human prediction markers service failed:\n" + res.message);
            }
            else {
                this.setMessage(res.message);
            }
        };
        this.arrowCallback = (success, position, orientation, message, senderInfo) => {
            if (success) {
                senderInfo.poseReceiveCB(position, orientation);
            }
            else {
                this.setMessage(message);
            }
        };
        this.setRobotGoalClicked = () => {
            this.startArrowControl(this.publishRobotGoal);
        };
        this.teleportRobotClicked = () => {
            this.startArrowControl(this.teleportRobot);
        };
        this.publishRobotGoal = (pose, direction) => {
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
            let goal = new ROSLIB.Goal({
                actionClient: this.moveBaseActionClient,
                goalMessage: new MoveBaseMsgs.MoveBaseGoal({
                    target_pose: robotGoal,
                }),
            });
            goal.send();
        };
        this.teleportRobot = (pose, direction) => {
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
            }
            else {
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
        };
        this.visualizationOptionsChanged = () => {
            this.disablePlannerVisualizationOptions();
            let showRobotGlobalPlan = document.getElementById("robot-global-plan-cb").checked;
            let showRobotLocalPlan = document.getElementById("robot-local-plan-cb").checked;
            let showRobotLocalPlanPoses = document.getElementById("robot-local-poses-cb").checked;
            let showHumanGlobalPlans = document.getElementById("human-global-plans-cb").checked;
            let showHumanLocalPlans = document.getElementById("human-local-plans-cb").checked;
            let showHumanLocalPlanPoses = document.getElementById("human-local-poses-cb").checked;
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
        };
        this.dynamicReconfigureInitialResponse = (res) => {
            let storedBoolName = JSON.parse(localStorage.getItem("selectedBool"));
            let boolSelectionList = document.getElementById("bool-param-name-sl");
            for (let boolParam of res.config.bools) {
                let boolOption = document.createElement("option");
                boolOption.text = boolParam.name;
                boolSelectionList.add(boolOption);
                if (boolOption.text === storedBoolName) {
                    boolOption.selected = true;
                }
            }
            let storedDoubleName = JSON.parse(localStorage.getItem("selectedDouble"));
            let doubleSelectionList = document.getElementById("double-param-name-sl");
            for (let doubleParam of res.config.doubles) {
                let doubleOption = document.createElement("option");
                doubleOption.text = doubleParam.name;
                doubleSelectionList.add(doubleOption);
                if (doubleOption.text === storedDoubleName) {
                    doubleOption.selected = true;
                }
            }
        };
        this.dynamicReconfigureResponse = (res) => {
            let boolSelectionList = document.getElementById("bool-param-name-sl");
            let boolParamName = boolSelectionList.options[boolSelectionList.selectedIndex].innerHTML;
            let boolParamFound = false;
            for (let boolParam of res.config.bools) {
                if (boolParam.name === "publish_robot_global_plan") {
                    document.getElementById("robot-global-plan-cb").checked = boolParam.value;
                }
                if (boolParam.name === "publish_robot_local_plan") {
                    document.getElementById("robot-local-plan-cb").checked = boolParam.value;
                }
                if (boolParam.name === "publish_robot_local_plan_poses") {
                    document.getElementById("robot-local-poses-cb").checked = boolParam.value;
                }
                if (boolParam.name === "publish_human_global_plans") {
                    document.getElementById("human-global-plans-cb").checked = boolParam.value;
                }
                if (boolParam.name === "publish_human_local_plans") {
                    document.getElementById("human-local-plans-cb").checked = boolParam.value;
                }
                if (boolParam.name === "publish_human_local_plan_poses") {
                    document.getElementById("human-local-poses-cb").checked = boolParam.value;
                }
                if (boolParam.name === boolParamName) {
                    document.getElementById("bool-param-value-cb").checked = boolParam.value;
                    boolParamFound = true;
                }
            }
            this.enablePlannerVisualizationOptions();
            if (!boolParamFound) {
                document.getElementById("bool-param-value-cb").checked = false;
            }
            let doubleSelectionList = document.getElementById("double-param-name-sl");
            let doubleParamName = doubleSelectionList.options[doubleSelectionList.selectedIndex].innerHTML;
            let doubleParamFound = false;
            for (let doubleParam of res.config.doubles) {
                if (doubleParam.name === doubleParamName) {
                    document.getElementById("double-param-value-ip").value = doubleParam.value.toString();
                    doubleParamFound = true;
                }
            }
            if (!doubleParamFound) {
                document.getElementById("double-param-value-ip").value = "NA";
            }
        };
        this.disablePlannerVisualizationOptions = () => {
            document.getElementById("robot-global-plan-cb").disabled = true;
            document.getElementById("robot-local-plan-cb").disabled = true;
            document.getElementById("robot-local-poses-cb").disabled = true;
            document.getElementById("human-global-plans-cb").disabled = true;
            document.getElementById("human-local-plans-cb").disabled = true;
            document.getElementById("human-local-poses-cb").disabled = true;
        };
        this.enablePlannerVisualizationOptions = () => {
            document.getElementById("robot-global-plan-cb").disabled = false;
            document.getElementById("robot-local-plan-cb").disabled = false;
            document.getElementById("robot-local-poses-cb").disabled = false;
            document.getElementById("human-global-plans-cb").disabled = false;
            document.getElementById("human-local-plans-cb").disabled = false;
            document.getElementById("human-local-poses-cb").disabled = false;
        };
        this.parameterChanged = () => {
            let boolSelectionList = document.getElementById("bool-param-name-sl");
            let boolParamName = boolSelectionList.options[boolSelectionList.selectedIndex].innerHTML;
            localStorage.setItem("selectedBool", JSON.stringify(boolParamName));
            let doubleSelectionList = document.getElementById("double-param-name-sl");
            let doubleParamName = doubleSelectionList.options[doubleSelectionList.selectedIndex].innerHTML;
            localStorage.setItem("selectedDouble", JSON.stringify(doubleParamName));
            let req = new DynamicReconfigure.ReconfigureRequest({
                config: new DynamicReconfigure.Config({}),
            });
            this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureResponse, this.failedServiceCallback);
        };
        this.doubleParameterSet = (event) => {
            event.preventDefault();
            if (event.keyCode !== 13) {
                return;
            }
            let selectionList = document.getElementById("double-param-name-sl");
            let paramName = selectionList.options[selectionList.selectedIndex].innerHTML;
            let paramInput = document.getElementById("double-param-value-ip").value;
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
                }
                else {
                    this.setMessage("Param value is NaN");
                }
            }
            else {
                this.setMessage("Please provide a parameter value");
            }
        };
        this.boolParameterSet = () => {
            let selectionList = document.getElementById("bool-param-name-sl");
            let paramName = selectionList.options[selectionList.selectedIndex].innerHTML;
            let paramValue = document.getElementById("bool-param-value-cb").checked;
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
        };
        this.populateExperiments = () => {
            let experimentsList = document.getElementById("experiments-sl");
            for (let experiment of this.experimets) {
                let option = document.createElement("option");
                option.text = experiment.name;
                experimentsList.add(option);
            }
        };
        this.experimentChanged = () => {
            let experimentsList = document.getElementById("experiments-sl");
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
        };
        this.startExperimentClicked = () => {
            let experimentsList = document.getElementById("experiments-sl");
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
                    setTimeout(() => this.moveHumansViz.setHumanGoal(experiment.human_id, experiment.human_goal), experiment.human_delay * 1000);
                    this.setMessage("Started " + selectedExperiment);
                    break;
                }
            }
        };
        this.setTRobotStartClicked = () => {
            this.startArrowControl(this.setTRobotStart);
        };
        this.setTRobotGoalClicked = () => {
            this.startArrowControl(this.setTRobotGoal);
        };
        this.setTRobotStart = (pose, direction) => {
            this.setTRobotStartGoal(pose, direction, true);
        };
        this.setTRobotGoal = (pose, direction) => {
            this.setTRobotStartGoal(pose, direction, false);
        };
        this.setTRobotStartGoal = (pose, direction, isStart) => {
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
            }
            else {
                this.robotPlanRequest.goal = selectedPose;
            }
            if (this.robotPlanRequest.start && this.robotPlanRequest.goal) {
                this.getPlanClient.callService(this.robotPlanRequest, this.getRobotPlanServiceResponse, this.failedServiceCallback);
            }
        };
        this.setTHumanStartClicked = () => {
            this.startArrowControl(this.setTHumanStart);
        };
        this.setTHumanGoalClicked = () => {
            this.startArrowControl(this.setTHumanGoal);
        };
        this.setTHumanDeleteClicked = () => {
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
        };
        this.THumanIDChanged = () => {
            this.humanPlanRequest.goal = null;
            this.humanPlanRequest.start = null;
        };
        this.setTHumanStart = (pose, direction) => {
            this.setTHumanStartGoal(pose, direction, true);
        };
        this.setTHumanGoal = (pose, direction) => {
            this.setTHumanStartGoal(pose, direction, false);
        };
        this.setTHumanStartGoal = (pose, direction, isStart) => {
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
                }
                else {
                    this.humanPlanRequest.goal = selectedPose;
                }
                if (this.humanPlanRequest.start && this.humanPlanRequest.goal) {
                    this.getPlanClient.callService(this.humanPlanRequest, this.getHumanPlanServiceResponse, this.failedServiceCallback);
                }
            }
        };
        this.getRobotPlanServiceResponse = (res) => {
            if (res.plan.poses.length > 0) {
                this.optimizeRequest.robot_plan = res.plan;
            }
            this.setMessage("got robot plan of size " + res.plan.poses.length);
        };
        this.getHumanPlanServiceResponse = (res) => {
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
        };
        this.optimizeClicked = () => {
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
                this.optimizePlansClient.callService(this.optimizeRequest, this.optimizeServiceResponse, this.failedServiceCallback);
            }
            else {
                this.setMessage("Please at least set a robot plan to optimize");
            }
        };
        this.optimizeServiceResponse = (res) => {
            this.setMessage("Optimizatin " + (res.success ? "succeded" : "failed") + "\n" + res.message);
        };
        this.config = config;
        this.ros = new ROSLIB.Ros();
        this.ros.on("error", this.rosConnectionError);
        this.ros.on("close", this.rosConnectionClosed);
        this.ros.on("connection", this.rosConnectionCreated);
        this.ros.connect(this.config.rosBridgeUrl);
    }
    startArrowControl(poseReceiveCB) {
        if (!this.gridClient.createArrowControl(this.arrowCallback, { poseReceiveCB: poseReceiveCB })) {
            this.setMessage("Please wait, map is being downloaded");
        }
        else {
            this.setMessage("Please choose a pose on the map");
        }
    }
    stopRobot() {
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
    setMessage(message) {
        document.getElementById("message-lb").value = message;
    }
    updateMessage(message) {
        document.getElementById("message-lb").value += message;
    }
    teleportRobotZero() {
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
    validateId(elementID) {
        let input = document.getElementById(elementID).value;
        if (input.length > 0) {
            let value = Number(input);
            if (!isNaN(value) && value % 1 === 0 && value >= 0) {
                return value;
            }
        }
        this.setMessage("Invalid human id");
        return -1;
    }
}
var MoveHumans;
(function (MoveHumans) {
    class HumanPose extends ROSLIB.Message {
        constructor(values) {
            super(values);
        }
    }
    MoveHumans.HumanPose = HumanPose;
    class HumanPoseArray extends ROSLIB.Message {
        constructor(values) {
            super(values);
        }
    }
    MoveHumans.HumanPoseArray = HumanPoseArray;
    class HumanUpdateRequest extends ROSLIB.ServiceRequest {
        constructor(values) {
            super(values);
        }
    }
    MoveHumans.HumanUpdateRequest = HumanUpdateRequest;
    class HumanUpdateResponse extends ROSLIB.ServiceResponse {
        constructor(values) {
            super(values);
        }
    }
    MoveHumans.HumanUpdateResponse = HumanUpdateResponse;
})(MoveHumans || (MoveHumans = {}));
var MoveHumans;
(function (MoveHumans) {
    class MoveHumansViz {
        constructor(config) {
            this.resizeViewer = () => {
                if (document.getElementById(this.viewerDivId) !== null) {
                    this.viewer.resize(document.getElementById(this.viewerDivId).clientWidth - 15, window.innerHeight - 18);
                }
            };
            this.resetSimulationClicked = () => {
                let req = new StdSrvs.TriggerRequest({});
                this.resetSimulationClient.callService(req, this.resetSimulationResponse, this.failedServiceCallback);
            };
            this.addHumanClicked = () => {
                this.startArrowControl("human-id-ip", this.addHumanRequest);
            };
            this.deleteHumanClicked = () => {
                let humanId = this.validateId("human-id-ip");
                if (humanId !== -1) {
                    this.deleteHumanRequest(humanId);
                }
            };
            this.addSubGoalClicked = () => {
                this.startArrowControl("human-id-ip", this.addSubgoalRequest);
            };
            this.updateGoalClicked = () => {
                this.startArrowControl("human-id-ip", this.updateGoalRequest);
            };
            this.teleportHumanClicked = () => {
                this.startArrowControl("human-id-ip", this.teleportHumanRequest);
            };
            this.followExternalPathsClicked = () => {
                if (document.getElementById("follow-plans-cb") !== null) {
                    let req = new StdSrvs.SetBoolRequest({
                        data: document.getElementById("follow-plans-cb").checked,
                    });
                    this.followExternalPathsClient.callService(req, this.followExternalPathsResponse, this.failedServiceCallback);
                }
            };
            this.failedServiceCallback = (message) => {
                this.setMessage("Service call failed:\n" + message);
            };
            this.resetSimulationResponse = (res) => {
                if (!res.success) {
                    this.setMessage("Failed to reset simulation:\n" + res.message);
                }
                else {
                    this.setMessage(res.message);
                }
            };
            this.followExternalPathsResponse = (res) => {
                if (!res.success) {
                    this.setMessage("Follow external paths service failed:\n" + res.message);
                }
                else {
                    this.setMessage(res.message);
                }
            };
            this.addHumanRequest = (id, pose, direction) => {
                let req = new MoveHumans.HumanUpdateRequest({
                    human_pose: this.createHumanPose(id, pose, direction),
                });
                this.addHumanClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
            };
            this.deleteHumanRequest = (id) => {
                let pose = new THREE.Vector3();
                let direction = new THREE.Quaternion();
                let req = new MoveHumans.HumanUpdateRequest({
                    human_pose: this.createHumanPose(id, pose, direction),
                });
                this.deleteHumanClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
            };
            this.addSubgoalRequest = (id, pose, direction) => {
                let req = new MoveHumans.HumanUpdateRequest({
                    human_pose: this.createHumanPose(id, pose, direction),
                });
                this.addSubgoalClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
            };
            this.updateGoalRequest = (id, pose, direction) => {
                let req = new MoveHumans.HumanUpdateRequest({
                    human_pose: this.createHumanPose(id, pose, direction),
                });
                this.updateGoalClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
            };
            this.teleportHumanRequest = (id, pose, direction) => {
                let req = new MoveHumans.HumanUpdateRequest({
                    human_pose: this.createHumanPose(id, pose, direction),
                });
                this.teleportHumanClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
            };
            this.arrowCallback = (success, position, orientation, message, senderInfo) => {
                this.enableHumanUpdateButtons();
                if (success) {
                    senderInfo.poseReceiveCB(senderInfo.id, position, orientation);
                }
                else {
                    this.setMessage(message);
                }
            };
            this.humanUpdateResponse = (res) => {
                if (!res.success) {
                    this.setMessage("Human update service failed:\n" + res.message);
                }
                else {
                    this.setMessage(res.message);
                }
            };
            this.controllerFrequencyUpdate = () => {
                if (document.getElementById("controller-freq-rg") !== null && document.getElementById("controller-freq-lb") !== null) {
                    let changedFrequency = Math.round(document.getElementById("controller-freq-rg")
                        .valueAsNumber * 10) / 10;
                    document.getElementById("controller-freq-lb").innerText = changedFrequency.toString();
                }
            };
            this.controllerFrequencyChanged = () => {
                if (document.getElementById("controller-freq-rg") !== null) {
                    let changedFrequency = Math.round(document.getElementById("controller-freq-rg")
                        .valueAsNumber * 10) / 10;
                    let req = new DynamicReconfigure.ReconfigureRequest({
                        config: new DynamicReconfigure.Config({
                            doubles: [new DynamicReconfigure.DoubleParameter({
                                    name: "controller_frequency",
                                    value: changedFrequency,
                                })],
                        }),
                    });
                    this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureResponse, this.failedServiceCallback);
                }
            };
            this.visualizationOptionsChanged = () => {
                if (document.getElementById("sim-human-markers-cb") !== null) {
                    let showHumanMarkers = document.getElementById("sim-human-markers-cb").checked;
                    let req = new DynamicReconfigure.ReconfigureRequest({
                        config: new DynamicReconfigure.Config({
                            bools: [new DynamicReconfigure.BoolParameter({
                                    name: "publish_human_markers",
                                    value: showHumanMarkers,
                                })],
                        }),
                    });
                    this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureResponse, this.failedServiceCallback);
                }
            };
            this.dynamicReconfigureResponse = (res) => {
                for (let doubleParam of res.config.doubles) {
                    if (doubleParam.name === "controller_frequency") {
                        if (document.getElementById("controller-freq-rg") !== null) {
                            document.getElementById("controller-freq-rg").valueAsNumber = doubleParam.value;
                        }
                        if (document.getElementById("controller-freq-lb") !== null) {
                            document.getElementById("controller-freq-lb").innerText = doubleParam.value.toString();
                        }
                    }
                }
                for (let boolParam of res.config.bools) {
                    if (boolParam.name === "publish_human_markers") {
                        if (document.getElementById("sim-human-markers-cb") !== null) {
                            document.getElementById("sim-human-markers-cb").checked = boolParam.value;
                        }
                    }
                }
            };
            this.plannerVisualizationOptionsChanged = () => {
                if (document.getElementById("sim-human-pplans-cb") !== null && document.getElementById("sim-human-poses-cb") !== null) {
                    let showHumanPaths = document.getElementById("sim-human-pplans-cb").checked;
                    let showHumanPathPoses = document.getElementById("sim-human-poses-cb").checked;
                    let req = new DynamicReconfigure.ReconfigureRequest({
                        config: new DynamicReconfigure.Config({
                            bools: [new DynamicReconfigure.BoolParameter({
                                    name: "publish_human_plans",
                                    value: showHumanPaths,
                                }),
                                new DynamicReconfigure.BoolParameter({
                                    name: "publish_human_poses",
                                    value: showHumanPathPoses,
                                })],
                        }),
                    });
                    this.plannerDynamicReconfigureClient.callService(req, this.plannerDynamicReconfigureResponse, this.failedServiceCallback);
                }
            };
            this.plannerDynamicReconfigureResponse = (res) => {
                for (let boolParam of res.config.bools) {
                    if (boolParam.name === "publish_human_plans") {
                        if (document.getElementById("sim-human-pplans-cb") !== null) {
                            document.getElementById("sim-human-pplans-cb").checked = boolParam.value;
                        }
                    }
                    if (boolParam.name === "publish_human_poses") {
                        if (document.getElementById("sim-human-poses-cb") !== null) {
                            document.getElementById("sim-human-poses-cb").checked = boolParam.value;
                        }
                    }
                }
            };
            this.controllerVisualizationOptionsChanged = () => {
                if (document.getElementById("sim-human-cplans-cb") !== null) {
                    let showHumanPaths = document.getElementById("sim-human-cplans-cb").checked;
                    let req = new DynamicReconfigure.ReconfigureRequest({
                        config: new DynamicReconfigure.Config({
                            bools: [new DynamicReconfigure.BoolParameter({
                                    name: "publish_plans",
                                    value: showHumanPaths,
                                })],
                        }),
                    });
                    this.controllerDynamicReconfigureClient.callService(req, this.controllerDynamicReconfigureResponse, this.failedServiceCallback);
                }
            };
            this.controllerDynamicReconfigureResponse = (res) => {
                for (let boolParam of res.config.bools) {
                    if (boolParam.name === "publish_plans") {
                        if (document.getElementById("sim-human-cplans-cb") !== null) {
                            document.getElementById("sim-human-cplans-cb").checked = boolParam.value;
                        }
                    }
                }
            };
            this.config = config;
            this.ros = this.config.ros;
            this.viewer = this.config.viewer;
            this.viewerDivId = this.config.viewerDivId;
            this.tfClient = this.config.tfClient;
            this.gridClient = this.config.gridClient;
            this.fixedFrame = this.config.fixedFrame;
            let humanControllerPaths = new ROS3DNAV.HumanPathArray({
                ros: this.ros,
                topic: this.config.humanControllerPathsTopic,
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
                color: 0x1E90FF,
                width: 1,
            });
            let humanPlannerPoses = new ROS3D.PoseArray({
                ros: this.ros,
                topic: this.config.humanPlannerPosesTopic,
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
            });
            let humansMarkers = new ROS3D.MarkerArrayClient({
                ros: this.ros,
                topic: this.config.humanMarkersTopic,
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
            });
            this.resetSimulationClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.resetSimulationService,
                serviceType: "std_srvs/Trigger",
            });
            if (document.getElementById("reset-simulation-bt") !== null) {
                document.getElementById("reset-simulation-bt").onclick = this.resetSimulationClicked;
            }
            this.addHumanClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.addHumanService,
                serviceType: "move_humans/HumanUpdate",
            });
            if (document.getElementById("add-human-bt") !== null) {
                document.getElementById("add-human-bt").onclick = this.addHumanClicked;
            }
            this.deleteHumanClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.deleteHumanService,
                serviceType: "move_humans/HumanUpdate",
            });
            if (document.getElementById("delete-human-bt") !== null) {
                document.getElementById("delete-human-bt").onclick = this.deleteHumanClicked;
            }
            this.addSubgoalClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.addSubgoalService,
                serviceType: "move_humans/HumanUpdate",
            });
            if (document.getElementById("") !== null) {
                document.getElementById("add-sub-goal-bt").onclick = this.addSubGoalClicked;
            }
            this.updateGoalClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.updateGoalService,
                serviceType: "move_humans/HumanUpdate",
            });
            if (document.getElementById("update-goal-bt") !== null) {
                document.getElementById("update-goal-bt").onclick = this.updateGoalClicked;
            }
            this.teleportHumanClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.teleportHumanService,
                serviceType: "move_humans/HumanUpdate",
            });
            if (document.getElementById("teleport-human-bt") !== null) {
                document.getElementById("teleport-human-bt").onclick = this.teleportHumanClicked;
            }
            this.followExternalPathsClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.followExternalPathsService,
                serviceType: "std_srvs/SetBool",
            });
            if (document.getElementById("follow-plans-cb") !== null) {
                document.getElementById("follow-plans-cb").onclick = this.followExternalPathsClicked;
            }
            this.dynamicReconfigureClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.dynamicReconfigureService,
                serviceType: "dynamic_reconfigure/Reconfigure",
            });
            if (document.getElementById("controller-freq-rg") !== null) {
                document.getElementById("controller-freq-rg").onchange = this.controllerFrequencyChanged;
            }
            if (document.getElementById("controller-freq-rg") !== null) {
                document.getElementById("controller-freq-rg").oninput = this.controllerFrequencyUpdate;
            }
            if (document.getElementById("sim-human-markers-cb") !== null) {
                document.getElementById("sim-human-markers-cb").onchange = this.visualizationOptionsChanged;
            }
            this.plannerDynamicReconfigureClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.plannerDynamicReconfigureService,
                serviceType: "dynamic_reconfigure/Reconfigure",
            });
            if (document.getElementById("sim-human-pplans-cb") !== null) {
                document.getElementById("sim-human-pplans-cb").onchange
                    = this.plannerVisualizationOptionsChanged;
            }
            if (document.getElementById("sim-human-poses-cb") !== null) {
                document.getElementById("sim-human-poses-cb").onchange
                    = this.plannerVisualizationOptionsChanged;
            }
            this.controllerDynamicReconfigureClient = new ROSLIB.Service({
                ros: this.ros,
                name: this.config.controllerDynamicReconfigureService,
                serviceType: "dynamic_reconfigure/Reconfigure",
            });
            if (document.getElementById("sim-human-cplans-cb") !== null) {
                document.getElementById("sim-human-cplans-cb").onchange
                    = this.controllerVisualizationOptionsChanged;
            }
            let req = new DynamicReconfigure.ReconfigureRequest({
                config: new DynamicReconfigure.Config({}),
            });
            this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureResponse, this.failedServiceCallback);
            this.plannerDynamicReconfigureClient.callService(req, this.plannerDynamicReconfigureResponse, this.failedServiceCallback);
            this.controllerDynamicReconfigureClient.callService(req, this.controllerDynamicReconfigureResponse, this.failedServiceCallback);
        }
        teleportHuman(id, pose) {
            let req = new MoveHumans.HumanUpdateRequest({
                human_pose: new MoveHumans.HumanPose({
                    human_id: id,
                    pose: new GeometryMsgs.PoseStamped({
                        header: new StdMsgs.Header({
                            seq: 1,
                            stamp: ROS3DNAV.Time.now(),
                            frame_id: this.fixedFrame,
                        }),
                        pose: pose,
                    }),
                }),
            });
            this.teleportHumanClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
        }
        setHumanGoal(id, pose) {
            let req = new MoveHumans.HumanUpdateRequest({
                human_pose: new MoveHumans.HumanPose({
                    human_id: id,
                    pose: new GeometryMsgs.PoseStamped({
                        header: new StdMsgs.Header({
                            seq: 1,
                            stamp: ROS3DNAV.Time.now(),
                            frame_id: this.fixedFrame,
                        }),
                        pose: pose,
                    }),
                }),
            });
            this.updateGoalClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
        }
        startArrowControl(humanIDField, poseReceiveCB) {
            let humanId = this.validateId(humanIDField);
            if (humanId !== -1) {
                if (!this.gridClient.createArrowControl(this.arrowCallback, { id: humanId, poseReceiveCB: poseReceiveCB })) {
                    this.setMessage("Please wait, map is being downloaded");
                }
                else {
                    this.setMessage("Please choose a pose on the map");
                    this.disableHumanUpdateButtons();
                }
            }
        }
        createHumanPose(id, pose, direction) {
            return new MoveHumans.HumanPose({
                human_id: id,
                pose: new GeometryMsgs.PoseStamped({
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
                }),
            });
        }
        setMessage(message) {
            if (document.getElementById("message-lb") !== null) {
                document.getElementById("message-lb").value = message;
            }
        }
        validateId(elementID) {
            if (document.getElementById(elementID) !== null) {
                let input = document.getElementById(elementID).value;
                if (input.length > 0) {
                    let value = Number(input);
                    if (!isNaN(value) && value % 1 === 0 && value >= 0) {
                        return value;
                    }
                }
                this.setMessage("Invalid human id");
                return -1;
            }
        }
        disableHumanUpdateButtons() {
            if (document.getElementById("add-human-bt") !== null) {
                document.getElementById("add-human-bt").disabled = true;
            }
            if (document.getElementById("delete-human-bt") !== null) {
                document.getElementById("delete-human-bt").disabled = true;
            }
            if (document.getElementById("add-sub-goal-bt") !== null) {
                document.getElementById("add-sub-goal-bt").disabled = true;
            }
            if (document.getElementById("update-goal-bt") !== null) {
                document.getElementById("update-goal-bt").disabled = true;
            }
        }
        enableHumanUpdateButtons() {
            if (document.getElementById("add-human-bt") !== null) {
                document.getElementById("add-human-bt").disabled = false;
            }
            if (document.getElementById("delete-human-bt") !== null) {
                document.getElementById("delete-human-bt").disabled = false;
            }
            if (document.getElementById("add-sub-goal-bt") !== null) {
                document.getElementById("add-sub-goal-bt").disabled = false;
            }
            if (document.getElementById("update-goal-bt") !== null) {
                document.getElementById("update-goal-bt").disabled = false;
            }
        }
    }
    MoveHumans.MoveHumansViz = MoveHumansViz;
})(MoveHumans || (MoveHumans = {}));
//# sourceMappingURL=hatebviz.js.map