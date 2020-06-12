var ROS3DNAV;
(function (ROS3DNAV) {
    class HumanPath extends THREE.Object3D {
        constructor(options) {
            super();
            this.options = options;
            this.pathReceived = (message) => {
                let previousLine = this.line;
                let lineGeometry = new THREE.Geometry();
                for (let i = 0; i < message.path.poses.length; i++) {
                    let v3 = new THREE.Vector3(message.path.poses[i].pose.position.x, message.path.poses[i].pose.position.y, message.path.poses[i].pose.position.z + this.zOffset);
                    lineGeometry.vertices.push(v3);
                }
                lineGeometry.computeLineDistances();
                let lineColor = typeof this.color === "number" ? this.color : this.color.getHex();
                let lineMaterial = new THREE.LineBasicMaterial({ color: lineColor, linewidth: this.width, overdraw: 0.5 });
                this.line = new THREE.Line(lineGeometry, lineMaterial);
                if (this.sn.frameID !== message.header.frame_id) {
                    this.sn.resubscribeTf(message.header.frame_id);
                }
                if (previousLine != null) {
                    this.sn.remove(previousLine);
                }
                this.sn.add(this.line);
            };
            this.rootObject = options.rootObject || new THREE.Object3D();
            this.color = options.color || new THREE.Color(0xcc00ff);
            this.width = options.width || 1;
            this.zOffset = options.zOffset || 0.05;
            this.line = new THREE.Line();
            this.sn = new ROS3DNAV.SceneNode({
                tfClient: options.tfClient,
                frameID: options.tfClient.fixedFrame,
                object: this.line,
            });
            this.rootObject.add(this.sn);
            let rosTopic = new ROSLIB.Topic({
                ros: options.ros,
                name: options.topic,
                messageType: "hanp_msgs/HumanPath",
            });
            rosTopic.subscribe(this.pathReceived);
        }
    }
    ROS3DNAV.HumanPath = HumanPath;
})(ROS3DNAV || (ROS3DNAV = {}));
var ROS3DNAV;
(function (ROS3DNAV) {
    class HumanPathArray extends THREE.Object3D {
        constructor(options) {
            super();
            this.options = options;
            this.pathArrayReceived = (message) => {
                let previousLines = this.lines;
                this.lines = new Array();
                for (let path of message.paths) {
                    let lineGeometry = new THREE.Geometry();
                    for (let i = 0; i < path.path.poses.length; i++) {
                        let v3 = new THREE.Vector3(path.path.poses[i].pose.position.x, path.path.poses[i].pose.position.y, path.path.poses[i].pose.position.z + this.zOffset);
                        lineGeometry.vertices.push(v3);
                    }
                    lineGeometry.computeLineDistances();
                    let lineColor = typeof this.color === "number" ? this.color : this.color.getHex();
                    let lineMaterial = new THREE.LineBasicMaterial({ color: lineColor, linewidth: this.width, overdraw: 0.5 });
                    this.lines.push(new THREE.Line(lineGeometry, lineMaterial));
                }
                if (this.sn.frameID !== message.header.frame_id) {
                    this.sn.resubscribeTf(message.header.frame_id);
                }
                if (previousLines != null) {
                    for (let line of previousLines) {
                        this.sn.remove(line);
                    }
                }
                for (let line of this.lines) {
                    this.sn.add(line);
                }
            };
            this.rootObject = options.rootObject || new THREE.Object3D();
            this.color = options.color || new THREE.Color(0xcc00ff);
            this.width = options.width || 1;
            this.zOffset = options.zOffset || 0.05;
            this.lines = new Array();
            this.sn = new ROS3DNAV.SceneNode({
                tfClient: options.tfClient,
                frameID: options.tfClient.fixedFrame,
                object: new THREE.Line(),
            });
            this.rootObject.add(this.sn);
            let rosTopic = new ROSLIB.Topic({
                ros: options.ros,
                name: options.topic,
                messageType: "hanp_msgs/HumanPathArray",
            });
            rosTopic.subscribe(this.pathArrayReceived);
        }
    }
    ROS3DNAV.HumanPathArray = HumanPathArray;
})(ROS3DNAV || (ROS3DNAV = {}));
var ROS3DNAV;
(function (ROS3DNAV) {
    class HumanTrajectory extends THREE.Object3D {
        constructor(options) {
            super();
            this.options = options;
            this.trajectoryReceived = (message) => {
                let previousLine = this.line;
                let lineGeometry = new THREE.Geometry();
                for (let i = 0; i < message.trajectory.points.length; i++) {
                    let v3 = new THREE.Vector3(message.trajectory.points[i].transform.translation.x, message.trajectory.points[i].transform.translation.y, message.trajectory.points[i].transform.translation.z + this.zOffset);
                    lineGeometry.vertices.push(v3);
                }
                lineGeometry.computeLineDistances();
                let lineColor = typeof this.color === "number" ? this.color : this.color.getHex();
                let lineMaterial = new THREE.LineBasicMaterial({ color: lineColor, linewidth: this.width, overdraw: 0.5 });
                this.line = new THREE.Line(lineGeometry, lineMaterial);
                if (this.sn.frameID !== message.header.frame_id) {
                    this.sn.resubscribeTf(message.header.frame_id);
                }
                if (previousLine != null) {
                    this.sn.remove(previousLine);
                }
                this.sn.add(this.line);
            };
            this.rootObject = options.rootObject || new THREE.Object3D();
            this.color = options.color || new THREE.Color(0xcc00ff);
            this.width = options.width || 1;
            this.zOffset = options.zOffset || 0.05;
            this.line = new THREE.Line();
            this.sn = new ROS3DNAV.SceneNode({
                tfClient: options.tfClient,
                frameID: options.tfClient.fixedFrame,
                object: this.line,
            });
            this.rootObject.add(this.sn);
            let rosTopic = new ROSLIB.Topic({
                ros: options.ros,
                name: options.topic,
                messageType: "hanp_msgs/HumanTrajectory",
            });
            rosTopic.subscribe(this.trajectoryReceived);
        }
    }
    ROS3DNAV.HumanTrajectory = HumanTrajectory;
})(ROS3DNAV || (ROS3DNAV = {}));
var ROS3DNAV;
(function (ROS3DNAV) {
    class HumanTrajectoryArray extends THREE.Object3D {
        constructor(options) {
            super();
            this.options = options;
            this.trajectoryArrayReceived = (message) => {
                let previousLines = this.lines;
                this.lines = new Array();
                for (let trajectory of message.trajectories) {
                    let lineGeometry = new THREE.Geometry();
                    for (let i = 0; i < trajectory.trajectory.points.length; i++) {
                        let v3 = new THREE.Vector3(trajectory.trajectory.points[i].transform.translation.x, trajectory.trajectory.points[i].transform.translation.y, trajectory.trajectory.points[i].transform.translation.z + this.zOffset);
                        lineGeometry.vertices.push(v3);
                    }
                    lineGeometry.computeLineDistances();
                    let lineColor = typeof this.color === "number" ? this.color : this.color.getHex();
                    let lineMaterial = new THREE.LineBasicMaterial({ color: lineColor, linewidth: this.width, overdraw: 0.5 });
                    this.lines.push(new THREE.Line(lineGeometry, lineMaterial));
                }
                if (this.sn.frameID !== message.header.frame_id) {
                    this.sn.resubscribeTf(message.header.frame_id);
                }
                if (previousLines != null) {
                    for (let line of previousLines) {
                        this.sn.remove(line);
                    }
                }
                for (let line of this.lines) {
                    this.sn.add(line);
                }
            };
            this.rootObject = options.rootObject || new THREE.Object3D();
            this.color = options.color || new THREE.Color(0xcc00ff);
            this.width = options.width || 1;
            this.zOffset = options.zOffset || 0.05;
            this.lines = new Array();
            this.sn = new ROS3DNAV.SceneNode({
                tfClient: options.tfClient,
                frameID: options.tfClient.fixedFrame,
                object: new THREE.Line(),
            });
            this.rootObject.add(this.sn);
            let rosTopic = new ROSLIB.Topic({
                ros: options.ros,
                name: options.topic,
                messageType: "hanp_msgs/HumanTrajectoryArray",
            });
            rosTopic.subscribe(this.trajectoryArrayReceived);
        }
    }
    ROS3DNAV.HumanTrajectoryArray = HumanTrajectoryArray;
})(ROS3DNAV || (ROS3DNAV = {}));
var ROS3DNAV;
(function (ROS3DNAV) {
    class OccupancyGridClient extends ROS3D.OccupancyGridClient {
        constructor(options) {
            super(options);
            this.onMouseDown = (event) => {
                this.mouseDownPoint = this.getMousePoint(event);
                removeEventListener("mousedown", this.onMouseDown);
                if (this.mouseDownPoint != null) {
                    addEventListener("mousemove", this.onMouseMove);
                    addEventListener("mouseup", this.onMouseUp);
                }
                else {
                    this.viewer.enableOrbitControls();
                    this.arrowCallback(false, new THREE.Vector3(), new THREE.Quaternion(), "Cannot calculate mousedown point", this.senderInfo);
                }
            };
            this.onMouseMove = (event) => {
                let mouseMovePoint = this.getMousePoint(event);
                if (mouseMovePoint != null) {
                    let direction = new THREE.Vector3().subVectors(mouseMovePoint, this.mouseDownPoint);
                    this.arrow.setLength(direction.length());
                    this.arrow.setDirection(direction.normalize());
                    if (!this.showingArrow) {
                        this.arrow.position = this.mouseDownPoint;
                        this.viewer.scene.add(this.arrow);
                    }
                    this.showingArrow = true;
                }
                else {
                    removeEventListener("mousemove", this.onMouseMove);
                    removeEventListener("mouseup", this.onMouseUp);
                    this.viewer.enableOrbitControls();
                    this.arrowCallback(false, new THREE.Vector3(), new THREE.Quaternion(), "Cannot calculate mousemove point", this.senderInfo);
                }
            };
            this.onMouseUp = (event) => {
                removeEventListener("mousemove", this.onMouseMove);
                removeEventListener("mouseup", this.onMouseUp);
                this.viewer.scene.remove(this.arrow);
                this.showingArrow = false;
                let mouseStopPoint = this.getMousePoint(event);
                if (mouseStopPoint != null) {
                    let theta = Math.atan2(mouseStopPoint.y - this.mouseDownPoint.y, mouseStopPoint.x - this.mouseDownPoint.x);
                    let quaternion = new THREE.Quaternion();
                    quaternion.setFromEuler(new THREE.Euler(0, 0, theta, "XYZ"));
                    quaternion.normalize();
                    this.viewer.enableOrbitControls();
                    this.arrowCallback(true, this.mouseDownPoint, quaternion, "", this.senderInfo);
                }
                else {
                    this.viewer.enableOrbitControls();
                    this.arrowCallback(false, new THREE.Vector3(), new THREE.Quaternion(), "Cannot calcualte mouseup point", this.senderInfo);
                }
            };
            this.viewer = options.viewer;
            this.mouseRaycaster = new THREE.Raycaster();
            this.projector = new THREE.Projector();
            this.mouseVector = new THREE.Vector3();
            let sourcePos = new THREE.Vector3(0, 0, 0);
            let targetPos = new THREE.Vector3(1, 0, 0);
            let direction = new THREE.Vector3().subVectors(targetPos, sourcePos);
            this.arrow = new THREE.ArrowHelper(direction.normalize(), sourcePos, direction.length(), 0x00ff00);
            this.showingArrow = false;
        }
        createArrowControl(arrowCallback, senderInfo) {
            if (!this.currentGrid) {
                return false;
            }
            this.viewer.disableOrbitControls();
            this.mouseDownPoint = null;
            addEventListener("mousedown", this.onMouseDown);
            this.senderInfo = senderInfo || null;
            this.arrowCallback = arrowCallback;
            return true;
        }
        getMousePoint(event) {
            this.mouseVector.x = (event.clientX / this.viewer.renderer.domElement.clientWidth) * 2 - 1;
            this.mouseVector.y = -(event.clientY / this.viewer.renderer.domElement.clientHeight) * 2 + 1;
            this.mouseVector.z = 0.0;
            this.projector.unprojectVector(this.mouseVector, this.viewer.camera);
            this.mouseRaycaster.set(this.viewer.camera.position, this.mouseVector.sub(this.viewer.camera.position).normalize());
            this.mouseRaycaster.linePrecision = 0.001;
            let intersects = this.mouseRaycaster.intersectObject(this.currentGrid, true);
            if (intersects.length > 0) {
                return new THREE.Vector3(intersects[0].point.x, intersects[0].point.y, 0.0);
            }
            else {
                return null;
            }
        }
    }
    ROS3DNAV.OccupancyGridClient = OccupancyGridClient;
})(ROS3DNAV || (ROS3DNAV = {}));
var ROS3DNAV;
(function (ROS3DNAV) {
    class Path extends THREE.Object3D {
        constructor(options) {
            super();
            this.options = options;
            this.pathReceived = (message) => {
                let previousLine = this.line;
                let lineGeometry = new THREE.Geometry();
                for (let i = 0; i < message.poses.length; i++) {
                    let v3 = new THREE.Vector3(message.poses[i].pose.position.x, message.poses[i].pose.position.y, message.poses[i].pose.position.z + this.zOffset);
                    lineGeometry.vertices.push(v3);
                }
                lineGeometry.computeLineDistances();
                let lineColor = typeof this.color === "number" ? this.color : this.color.getHex();
                let lineMaterial = new THREE.LineBasicMaterial({ color: lineColor, linewidth: this.width, overdraw: 0.5 });
                this.line = new THREE.Line(lineGeometry, lineMaterial);
                if (this.sn.frameID !== message.header.frame_id) {
                    this.sn.resubscribeTf(message.header.frame_id);
                }
                if (previousLine != null) {
                    this.sn.remove(previousLine);
                }
                this.sn.add(this.line);
            };
            this.rootObject = options.rootObject || new THREE.Object3D();
            this.color = options.color || new THREE.Color(0xcc00ff);
            this.width = options.width || 1;
            this.zOffset = options.zOffset || 0.05;
            this.line = new THREE.Line();
            this.sn = new ROS3DNAV.SceneNode({
                tfClient: options.tfClient,
                frameID: options.tfClient.fixedFrame,
                object: this.line,
            });
            this.rootObject.add(this.sn);
            let rosTopic = new ROSLIB.Topic({
                ros: options.ros,
                name: options.topic,
                messageType: "nav_msgs/Path",
            });
            rosTopic.subscribe(this.pathReceived);
        }
    }
    ROS3DNAV.Path = Path;
})(ROS3DNAV || (ROS3DNAV = {}));
var ROS3DNAV;
(function (ROS3DNAV) {
    class Trajectory extends THREE.Object3D {
        constructor(options) {
            super();
            this.options = options;
            this.trajectoryReceived = (message) => {
                let previousLine = this.line;
                let lineGeometry = new THREE.Geometry();
                for (let i = 0; i < message.points.length; i++) {
                    let v3 = new THREE.Vector3(message.points[i].transform.translation.x, message.points[i].transform.translation.y, message.points[i].transform.translation.z + this.zOffset);
                    lineGeometry.vertices.push(v3);
                }
                lineGeometry.computeLineDistances();
                let lineColor = typeof this.color === "number" ? this.color : this.color.getHex();
                let lineMaterial = new THREE.LineBasicMaterial({ color: lineColor, linewidth: this.width, overdraw: 0.5 });
                this.line = new THREE.Line(lineGeometry, lineMaterial);
                if (this.sn.frameID !== message.header.frame_id) {
                    this.sn.resubscribeTf(message.header.frame_id);
                }
                if (previousLine != null) {
                    this.sn.remove(previousLine);
                }
                this.sn.add(this.line);
            };
            this.rootObject = options.rootObject || new THREE.Object3D();
            this.color = options.color || new THREE.Color(0xcc00ff);
            this.width = options.width || 1;
            this.zOffset = options.zOffset || 0.05;
            this.line = new THREE.Line();
            this.sn = new ROS3DNAV.SceneNode({
                tfClient: options.tfClient,
                frameID: options.tfClient.fixedFrame,
                object: this.line,
            });
            this.rootObject.add(this.sn);
            let rosTopic = new ROSLIB.Topic({
                ros: options.ros,
                name: options.topic,
                messageType: "hanp_msgs/Trajectory",
            });
            rosTopic.subscribe(this.trajectoryReceived);
        }
    }
    ROS3DNAV.Trajectory = Trajectory;
})(ROS3DNAV || (ROS3DNAV = {}));
var ROS3DNAV;
(function (ROS3DNAV) {
    class TrajectoryArray extends THREE.Object3D {
        constructor(options) {
            super();
            this.options = options;
            this.trajectoryArrayReceived = (message) => {
                let previousLines = this.lines;
                this.lines = new Array();
                for (let trajectory of message.trajectories) {
                    let lineGeometry = new THREE.Geometry();
                    for (let i = 0; i < trajectory.points.length; i++) {
                        let v3 = new THREE.Vector3(trajectory.points[i].transform.translation.x, trajectory.points[i].transform.translation.y, trajectory.points[i].transform.translation.z + this.zOffset);
                        lineGeometry.vertices.push(v3);
                    }
                    lineGeometry.computeLineDistances();
                    let lineColor = typeof this.color === "number" ? this.color : this.color.getHex();
                    let lineMaterial = new THREE.LineBasicMaterial({ color: lineColor, linewidth: this.width, overdraw: 0.5 });
                    this.lines.push(new THREE.Line(lineGeometry, lineMaterial));
                }
                if (this.sn.frameID !== message.header.frame_id) {
                    this.sn.resubscribeTf(message.header.frame_id);
                }
                if (previousLines != null) {
                    for (let line of previousLines) {
                        this.sn.remove(line);
                    }
                }
                for (let line of this.lines) {
                    this.sn.add(line);
                }
            };
            this.rootObject = options.rootObject || new THREE.Object3D();
            this.color = options.color || new THREE.Color(0xcc00ff);
            this.width = options.width || 1;
            this.zOffset = options.zOffset || 0.05;
            this.lines = new Array();
            this.sn = new ROS3DNAV.SceneNode({
                tfClient: options.tfClient,
                frameID: options.tfClient.fixedFrame,
                object: new THREE.Line(),
            });
            this.rootObject.add(this.sn);
            let rosTopic = new ROSLIB.Topic({
                ros: options.ros,
                name: options.topic,
                messageType: "hanp_msgs/TrajectoryArray",
            });
            rosTopic.subscribe(this.trajectoryArrayReceived);
        }
    }
    ROS3DNAV.TrajectoryArray = TrajectoryArray;
})(ROS3DNAV || (ROS3DNAV = {}));
var ROS3DNAV;
(function (ROS3DNAV) {
    class Time {
        constructor(secs, nsecs) {
            this.time = new Date(secs * 1000 + Math.floor(nsecs * 0.000001));
        }
        static now() {
            let now = (new Date()).getTime();
            return { secs: Math.floor(now / 1000), nsecs: now % 1000 };
        }
        toSec() {
            return this.time.getSeconds() + this.time.getMilliseconds() * 0.001;
        }
    }
    ROS3DNAV.Time = Time;
})(ROS3DNAV || (ROS3DNAV = {}));
var ROS3DNAV;
(function (ROS3DNAV) {
    class SceneNode extends ROS3D.SceneNode {
        resubscribeTf(frameID) {
            this.tfClient.unsubscribe(this.frameID, this.tfUpdate);
            this.frameID = frameID;
            this.tfClient.subscribe(this.frameID, this.tfUpdate);
        }
    }
    ROS3DNAV.SceneNode = SceneNode;
})(ROS3DNAV || (ROS3DNAV = {}));
var ROS3DNAV;
(function (ROS3DNAV) {
    class Viewer extends ROS3D.Viewer {
        constructor() {
            super(...arguments);
            this.orbitControlsEnabled = true;
        }
        disableOrbitControls() {
            if (this.orbitControlsEnabled) {
                this.lastAutoRotateSpeed = this.cameraControls.autoRotateSpeed;
                this.lastUserRotateSpeed = this.cameraControls.userRotateSpeed;
                this.cameraControls.autoRotateSpeed = 0;
                this.cameraControls.userRotateSpeed = 0;
                this.orbitControlsEnabled = false;
            }
        }
        enableOrbitControls() {
            if (!this.orbitControlsEnabled) {
                this.cameraControls.autoRotateSpeed = this.lastAutoRotateSpeed;
                this.cameraControls.userRotateSpeed = this.lastUserRotateSpeed;
                this.orbitControlsEnabled = true;
            }
        }
    }
    ROS3DNAV.Viewer = Viewer;
})(ROS3DNAV || (ROS3DNAV = {}));
//# sourceMappingURL=ros3dnav.js.map