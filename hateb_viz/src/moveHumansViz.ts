namespace MoveHumans {
  export interface MoveHumansConfig {
    ros: ROSLIB.Ros;
    viewer: ROS3DNAV.Viewer;
    viewerDivId: string;
    tfClient: ROSLIB.TFClient;
    gridClient: ROS3DNAV.OccupancyGridClient;
    fixedFrame: string;
    humanPlannerPathsTopic: string;
    humanPlannerPosesTopic: string;
    humanControllerPathsTopic: string;
    humanMarkersTopic: string;
    resetSimulationService: string;
    addHumanService: string;
    deleteHumanService: string;
    addSubgoalService: string;
    updateGoalService: string;
    teleportHumanService: string;
    followExternalPathsService: string;
    dynamicReconfigureService: string;
    plannerDynamicReconfigureService: string;
    controllerDynamicReconfigureService: string;
  }

  export class MoveHumansViz {
    private config: MoveHumansConfig;
    private ros: ROSLIB.Ros;
    private viewer: ROS3DNAV.Viewer;
    private viewerDivId: string;
    private tfClient: ROSLIB.TFClient;
    private gridClient: ROS3DNAV.OccupancyGridClient;

    private resetSimulationClient: ROSLIB.Service;
    private addHumanClient: ROSLIB.Service;
    private deleteHumanClient: ROSLIB.Service;
    private addSubgoalClient: ROSLIB.Service;
    private updateGoalClient: ROSLIB.Service;
    private teleportHumanClient: ROSLIB.Service;
    private followExternalPathsClient: ROSLIB.Service;
    private dynamicReconfigureClient: ROSLIB.Service;
    private plannerDynamicReconfigureClient: ROSLIB.Service;
    private controllerDynamicReconfigureClient: ROSLIB.Service;

    private fixedFrame: string;

    constructor(config: MoveHumansConfig) {

      this.config = config;

      this.ros = this.config.ros;
      this.viewer = this.config.viewer;
      this.viewerDivId = this.config.viewerDivId;
      this.tfClient = this.config.tfClient;
      this.gridClient = this.config.gridClient;
      this.fixedFrame = this.config.fixedFrame;

      // setup human paths clients
      // let humanPlannerPaths = new ROS3DNAV.HumanPathArray({
      //   ros: this.ros,
      //   topic: this.config.humanPlannerPathsTopic,
      //   tfClient: this.tfClient,
      //   rootObject: this.viewer.scene,
      //   color: 0x7B0D0D,
      //   width: 1,
      // });
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

      // setup human marker client
      let humansMarkers = new ROS3D.MarkerArrayClient({
        ros: this.ros,
        topic: this.config.humanMarkersTopic,
        tfClient: this.tfClient,
        rootObject: this.viewer.scene,
      });

      // setup services
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
        (<HTMLInputElement>document.getElementById("follow-plans-cb")).onclick = this.followExternalPathsClicked;
      }

      this.dynamicReconfigureClient = new ROSLIB.Service({
        ros: this.ros,
        name: this.config.dynamicReconfigureService,
        serviceType: "dynamic_reconfigure/Reconfigure",
      });
      if (document.getElementById("controller-freq-rg") !== null) {
        (<HTMLInputElement>document.getElementById("controller-freq-rg")).onchange = this.controllerFrequencyChanged;
      }
      if (document.getElementById("controller-freq-rg") !== null) {
        (<HTMLInputElement>document.getElementById("controller-freq-rg")).oninput = this.controllerFrequencyUpdate;
      }
      if (document.getElementById("sim-human-markers-cb") !== null) {
        (<HTMLInputElement>document.getElementById("sim-human-markers-cb")).onchange = this.visualizationOptionsChanged;
      }

      this.plannerDynamicReconfigureClient = new ROSLIB.Service({
        ros: this.ros,
        name: this.config.plannerDynamicReconfigureService,
        serviceType: "dynamic_reconfigure/Reconfigure",
      });
      if (document.getElementById("sim-human-pplans-cb") !== null) {
        (<HTMLInputElement>document.getElementById("sim-human-pplans-cb")).onchange
          = this.plannerVisualizationOptionsChanged;
      }
      if (document.getElementById("sim-human-poses-cb") !== null) {
        (<HTMLInputElement>document.getElementById("sim-human-poses-cb")).onchange
          = this.plannerVisualizationOptionsChanged;
      }
      // if (document.getElementById("sim-potential-cb") !== null) {
      //   (<HTMLInputElement>document.getElementById("sim-potential-cb")).onchange
      //     = this.plannerVisualizationOptionsChanged;
      // }

      this.controllerDynamicReconfigureClient = new ROSLIB.Service({
        ros: this.ros,
        name: this.config.controllerDynamicReconfigureService,
        serviceType: "dynamic_reconfigure/Reconfigure",
      });
      if (document.getElementById("sim-human-cplans-cb") !== null) {
        (<HTMLInputElement>document.getElementById("sim-human-cplans-cb")).onchange
          = this.controllerVisualizationOptionsChanged;
      }

      // get all dynamic reconfigurable parameters
      let req = new DynamicReconfigure.ReconfigureRequest({
        config: new DynamicReconfigure.Config({}),
      });
      this.dynamicReconfigureClient.callService(req, this.dynamicReconfigureResponse, this.failedServiceCallback);
      this.plannerDynamicReconfigureClient.callService(req, this.plannerDynamicReconfigureResponse,
        this.failedServiceCallback);
      this.controllerDynamicReconfigureClient.callService(req, this.controllerDynamicReconfigureResponse,
        this.failedServiceCallback);
    }

    public resizeViewer = () => {
      if (document.getElementById(this.viewerDivId) !== null) {
        this.viewer.resize(document.getElementById(this.viewerDivId).clientWidth - 15,
          window.innerHeight - 18);
      }
    }

    public teleportHuman(id: number, pose: GeometryMsgs.Pose) {
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

    public setHumanGoal(id: number, pose: GeometryMsgs.Pose) {
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

    private resetSimulationClicked = () => {
      let req = new StdSrvs.TriggerRequest({});
      this.resetSimulationClient.callService(req, this.resetSimulationResponse, this.failedServiceCallback);
    }

    private addHumanClicked = () => {
      this.startArrowControl("human-id-ip", this.addHumanRequest);
    }

    private deleteHumanClicked = () => {
      let humanId = this.validateId("human-id-ip");
      if (humanId !== -1) {
        this.deleteHumanRequest(humanId);
      }
    }

    private addSubGoalClicked = () => {
      this.startArrowControl("human-id-ip", this.addSubgoalRequest);
    }

    private updateGoalClicked = () => {
      this.startArrowControl("human-id-ip", this.updateGoalRequest);
    }

    private teleportHumanClicked = () => {
      this.startArrowControl("human-id-ip", this.teleportHumanRequest);
    }

    private followExternalPathsClicked = () => {
      if (document.getElementById("follow-plans-cb") !== null) {
        let req = new StdSrvs.SetBoolRequest({
          data: (<HTMLInputElement>document.getElementById("follow-plans-cb")).checked,
        });
        this.followExternalPathsClient.callService(req, this.followExternalPathsResponse, this.failedServiceCallback);
      }
    }

    private failedServiceCallback = (message: string) => {
      this.setMessage("Service call failed:\n" + message);
    }

    private resetSimulationResponse = (res: StdSrvs.TriggerResponse) => {
      if (!res.success) {
        this.setMessage("Failed to reset simulation:\n" + res.message);
      } else {
        this.setMessage(res.message);
      }
    }

    private followExternalPathsResponse = (res: StdSrvs.SetBoolResponse) => {
      if (!res.success) {
        this.setMessage("Follow external paths service failed:\n" + res.message);
      } else {
        this.setMessage(res.message);
      }
    }

    private startArrowControl(humanIDField: string, poseReceiveCB: any) {
      let humanId = this.validateId(humanIDField);
      if (humanId !== -1) {
        if (!this.gridClient.createArrowControl(this.arrowCallback, { id: humanId, poseReceiveCB: poseReceiveCB })) {
          this.setMessage("Please wait, map is being downloaded");
        } else {
          this.setMessage("Please choose a pose on the map");
          this.disableHumanUpdateButtons();
        }
      }
    }

    private addHumanRequest = (id: number, pose: THREE.Vector3, direction: THREE.Quaternion) => {
      let req = new MoveHumans.HumanUpdateRequest({
        human_pose: this.createHumanPose(id, pose, direction),
      });
      this.addHumanClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
    }

    private deleteHumanRequest = (id: number) => {
      let pose = new THREE.Vector3();
      let direction = new THREE.Quaternion();
      let req = new MoveHumans.HumanUpdateRequest({
        human_pose: this.createHumanPose(id, pose, direction),
      });
      this.deleteHumanClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
    }

    private addSubgoalRequest = (id: number, pose: THREE.Vector3, direction: THREE.Quaternion) => {
      let req = new MoveHumans.HumanUpdateRequest({
        human_pose: this.createHumanPose(id, pose, direction),
      });
      this.addSubgoalClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
    }

    private updateGoalRequest = (id: number, pose: THREE.Vector3, direction: THREE.Quaternion) => {
      let req = new MoveHumans.HumanUpdateRequest({
        human_pose: this.createHumanPose(id, pose, direction),
      });
      this.updateGoalClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
    }

    private teleportHumanRequest = (id: number, pose: THREE.Vector3, direction: THREE.Quaternion) => {
      let req = new MoveHumans.HumanUpdateRequest({
        human_pose: this.createHumanPose(id, pose, direction),
      });
      this.teleportHumanClient.callService(req, this.humanUpdateResponse, this.failedServiceCallback);
    }

    private arrowCallback = (success: boolean, position?: THREE.Vector3, orientation?: THREE.Quaternion,
      message?: string, senderInfo?: any) => {
      this.enableHumanUpdateButtons();
      if (success) {
        senderInfo.poseReceiveCB(senderInfo.id, position, orientation);
      } else {
        this.setMessage(message);
      }
    }

    private humanUpdateResponse = (res: MoveHumans.HumanUpdateResponse) => {
      if (!res.success) {
        this.setMessage("Human update service failed:\n" + res.message);
      } else {
        this.setMessage(res.message);
      }
    }

    private createHumanPose(id: number, pose: THREE.Vector3, direction: THREE.Quaternion) {
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

    private setMessage(message: string) {

      if (document.getElementById("message-lb") !== null) {
        (<HTMLInputElement>document.getElementById("message-lb")).value = message;
      }
    }

    private validateId(elementID: string) {
      if (document.getElementById(elementID) !== null) {
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
    }

    private disableHumanUpdateButtons() {

      if (document.getElementById("add-human-bt") !== null) {
        (<HTMLButtonElement>document.getElementById("add-human-bt")).disabled = true;
      }
      if (document.getElementById("delete-human-bt") !== null) {
        (<HTMLButtonElement>document.getElementById("delete-human-bt")).disabled = true;
      }
      if (document.getElementById("add-sub-goal-bt") !== null) {
        (<HTMLButtonElement>document.getElementById("add-sub-goal-bt")).disabled = true;
      }
      if (document.getElementById("update-goal-bt") !== null) {
        (<HTMLButtonElement>document.getElementById("update-goal-bt")).disabled = true;
      }
    }

    private enableHumanUpdateButtons() {
      if (document.getElementById("add-human-bt") !== null) {
        (<HTMLButtonElement>document.getElementById("add-human-bt")).disabled = false;
      }
      if (document.getElementById("delete-human-bt") !== null) {
        (<HTMLButtonElement>document.getElementById("delete-human-bt")).disabled = false;
      }
      if (document.getElementById("add-sub-goal-bt") !== null) {
        (<HTMLButtonElement>document.getElementById("add-sub-goal-bt")).disabled = false;
      }
      if (document.getElementById("update-goal-bt") !== null) {
        (<HTMLButtonElement>document.getElementById("update-goal-bt")).disabled = false;
      }
    }

    private controllerFrequencyUpdate = () => {
      if (document.getElementById("controller-freq-rg") !== null && document.getElementById("controller-freq-lb") !== null) {
        let changedFrequency = Math.round((<HTMLInputElement>document.getElementById("controller-freq-rg"))
          .valueAsNumber * 10) / 10;
        (<HTMLButtonElement>document.getElementById("controller-freq-lb")).innerText = changedFrequency.toString();
      }
    }

    private controllerFrequencyChanged = () => {
      if (document.getElementById("controller-freq-rg") !== null) {
        let changedFrequency = Math.round((<HTMLInputElement>document.getElementById("controller-freq-rg"))
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
    }

    private visualizationOptionsChanged = () => {
      if (document.getElementById("sim-human-markers-cb") !== null) {
        let showHumanMarkers = (<HTMLInputElement>document.getElementById("sim-human-markers-cb")).checked;
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
    }

    private dynamicReconfigureResponse = (res: DynamicReconfigure.ReconfigureResponse) => {
      for (let doubleParam of res.config.doubles) {
        if (doubleParam.name === "controller_frequency") {
          if (document.getElementById("controller-freq-rg") !== null) {
            (<HTMLInputElement>document.getElementById("controller-freq-rg")).valueAsNumber = doubleParam.value;
          }
          if (document.getElementById("controller-freq-lb") !== null) {
            (<HTMLButtonElement>document.getElementById("controller-freq-lb")).innerText = doubleParam.value.toString();
          }
        }
      }
      for (let boolParam of res.config.bools) {
        if (boolParam.name === "publish_human_markers") {
          if (document.getElementById("sim-human-markers-cb") !== null) {
            (<HTMLInputElement>document.getElementById("sim-human-markers-cb")).checked = boolParam.value;
          }
        }
      }
    }

    private plannerVisualizationOptionsChanged = () => {
      if (document.getElementById("sim-human-pplans-cb") !== null && document.getElementById("sim-human-poses-cb") !== null) {
        let showHumanPaths = (<HTMLInputElement>document.getElementById("sim-human-pplans-cb")).checked;
        let showHumanPathPoses = (<HTMLInputElement>document.getElementById("sim-human-poses-cb")).checked;
        // let showPlannerPotential = (<HTMLInputElement>document.getElementById("sim-potential-id")).checked;
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
            // new DynamicReconfigure.BoolParameter({
            //   name: "publish_potential",
            //   value: showPlannerPotential,
            // })]
          }),
        });
        this.plannerDynamicReconfigureClient.callService(req, this.plannerDynamicReconfigureResponse,
          this.failedServiceCallback);
      }
    }

    private plannerDynamicReconfigureResponse = (res: DynamicReconfigure.ReconfigureResponse) => {
      for (let boolParam of res.config.bools) {
        if (boolParam.name === "publish_human_plans") {
          if (document.getElementById("sim-human-pplans-cb") !== null) {
            (<HTMLInputElement>document.getElementById("sim-human-pplans-cb")).checked = boolParam.value;
          }
        }
        if (boolParam.name === "publish_human_poses") {
          if (document.getElementById("sim-human-poses-cb") !== null) {
            (<HTMLInputElement>document.getElementById("sim-human-poses-cb")).checked = boolParam.value;
          }
        }
        // if (boolParam.name === "publish_potential") {
        //   if (document.getElementById("sim-potential-id") !== null) {
        //     (<HTMLInputElement>document.getElementById("sim-potential-id")).checked = boolParam.value;
        //   }
        // }
      }
    }

    private controllerVisualizationOptionsChanged = () => {
      if (document.getElementById("sim-human-cplans-cb") !== null) {
        let showHumanPaths = (<HTMLInputElement>document.getElementById("sim-human-cplans-cb")).checked;
        let req = new DynamicReconfigure.ReconfigureRequest({
          config: new DynamicReconfigure.Config({
            bools: [new DynamicReconfigure.BoolParameter({
              name: "publish_plans",
              value: showHumanPaths,
            })],
          }),
        });
        this.controllerDynamicReconfigureClient.callService(req, this.controllerDynamicReconfigureResponse,
          this.failedServiceCallback);
      }
    }

    private controllerDynamicReconfigureResponse = (res: DynamicReconfigure.ReconfigureResponse) => {
      for (let boolParam of res.config.bools) {
        if (boolParam.name === "publish_plans") {
          if (document.getElementById("sim-human-cplans-cb") !== null) {
            (<HTMLInputElement>document.getElementById("sim-human-cplans-cb")).checked = boolParam.value;
          }
        }
      }
    }
  }
}
