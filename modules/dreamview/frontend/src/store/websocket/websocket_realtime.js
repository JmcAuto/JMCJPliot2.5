import STORE from "store";
import RENDERER from "renderer";
//import MAP_NAVIGATOR from "components/Navigation/MapNavigator";
import Worker from 'utils/webworker.js';
import { UTMToWGS84 } from "utils/coordinate_converter";
import { WGS84ToUTM } from "utils/coordinate_converter";


export default class RosWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.simWorldUpdatePeriodMs = 100;
        this.simWorldLastUpdateTimestamp = 0;
        this.mapUpdatePeriodMs = 1000;
        this.mapLastUpdateTimestamp = 0;
        this.updatePOI = true;
        this.routingTime = undefined;
        this.currentMode = null;
        this.worker = new Worker();
        //this.preStatus = "";
        //开子线程
    }

    initialize() {
        try {
            this.websocket = new WebSocket(this.serverAddr);
            //console.log(this.serverAddr);
            this.websocket.binaryType = "arraybuffer";
            //定义收到的数据类型为二进制数据，建立连接之后修改传输类型，以字符数组形式进行数据传输。前端和后端双向(看代码应该是客户端收到了二进制数据数组)
        } catch (error) {
            //console.error("Failed to establish a connection: " + error);
            setTimeout(() => {
                this.initialize();
            }, 1000);
            return;
        }
        this.websocket.onmessage = event => {
            //event里包含后端推送的数据
            //console.log("your eventdata is: ");
            //console.log(typeof(eventdata));
            //console.log(event.data);
            this.worker.postMessage({
                //主线程给子线程发送信息，data在event事件中
                source: 'realtime',
                data: event.data,
            });
        };
        //主线程接收来自子线程的消息
        this.worker.onmessage = event => {
            const message = event.data;
            switch (message.type) {
                //HMI在连接建立的时候就会主动推送消息给前端
                case "HMIConfig":
                    STORE.hmi.initialize(message.data);
                    break;
                case "HMIStatus":
                    STORE.hmi.updateStatus(message.data);
                    RENDERER.updateGroundImage(STORE.hmi.currentMap);
                    break;
                case "ParkingPointCheck":
                    if (message.status === 1) {
                        this.requestRoute(message.start, [], message.end, message.status);
                    }
                    else {
                        const confirm_ = confirm("Find parking space automatically!");
                        if (confirm_ === true) {
                            this.requestRoute(message.start, [], message.end, message.status);
                        }
                    }
                    break;
                case "VehicleParam":
                    STORE.hmi.updateVehicleParam(message.data);
                    break;
                case "SimControlStatus":
                    STORE.setOptionStatus('simControlEnabled', message.enabled);
                    break;
                case "SimWorldUpdate":
                    this.checkMessage(message);
                    if (message.parkingStatus === "SUCCEED"
                    && message.parkingStatus !== this.preStatus) {
                        this.changeDrivingMode("COMPLETE_MANUAL");
                        console.log("Willing to change to manual");
                        alert("MISSION COMPLETED!");
                    }
                    else if (message.parkingStatus === "FAILED"
                    && message.parkingStatus !== this.preStatus) {
                        this.changeDrivingMode("COMPLETE_MANUAL");
                        console.log("Willing to change to manual");
                        alert("MISSION FAILED");
                    }
                    this.preStatus = message.parkingStatus;

                   // console.log("the preStatus is: ");
                   // console.log(this.preStatus);
                   // console.log(typeof(this.preStatus));
                   // console.log(typeof(message.parkingStatus));

                    /*
                    if (message.parkingStatus === "SUCCEED"
                    && message.parkingStatus !== this.preStatus) {
                        this.changeDrivingMode("COMPLETE_MANUAL");
                        console.log("Willing to change to manual");
                        alert("MISSION COMPLETED!");
                    }
                    else if (message.parkingStatus === "FAILED"
                    && message.parkingStatus !== this.preStatus) {
                        this.changeDrivingMode("COMPLETE_MANUAL");
                        console.log("Willing to change to manual");
                        alert("MISSION FAILED");
                    }else if(message.parkingStatus === "SHORT_SUMMON_SUCCEED"
                    && message.parkingStatus !== this.preStatus) {
                        this.changeDrivingMode("COMPLETE_MANUAL");
                        console.log("Willing to change to manual");
                        alert("short summon succeed!");
                    }

                    this.preStatus = message.parkingStatus;
                    */

                    const updateCoordination = (this.currentMode !== STORE.hmi.currentMode);
                    this.currentMode = STORE.hmi.currentMode;
                    if (STORE.hmi.inNavigationMode) {
                        // In navigation mode, relative map is set and the relative position
                        // of auto driving car is (0, 0). Absolute position of the car
                        // only needed in MAP_NAVIGATOR.
                        //if (MAP_NAVIGATOR.isInitialized()) {
                        //    MAP_NAVIGATOR.update(message);
                        //}
                        message.autoDrivingCar.positionX = 0;
                        message.autoDrivingCar.positionY = 0;
                        message.autoDrivingCar.heading = 0;

                        RENDERER.coordinates.setSystem("FLU");
                        this.mapUpdatePeriodMs = 100;
                    } else {
                        RENDERER.coordinates.setSystem("ENU");
                        this.mapUpdatePeriodMs = 1000;
                    }

                    STORE.updateTimestamp(message.timestamp);
                    STORE.updateModuleDelay(message);
                    RENDERER.maybeInitializeOffest(
                        message.autoDrivingCar.positionX,
                        message.autoDrivingCar.positionY,
                        updateCoordination);
                    STORE.meters.update(message);
                    STORE.monitor.update(message);
                    STORE.trafficSignal.update(message);
                    STORE.hmi.update(message);
                    RENDERER.updateWorld(message);
                    this.updateMapIndex(message);
                    if (STORE.options.showPNCMonitor) {
                        STORE.planningData.update(message);
                        STORE.controlData.update(message, STORE.hmi.vehicleParam);
                    }
                    if (this.routingTime !== message.routingTime) {
                        // A new routing needs to be fetched from backend.
                        this.requestRoutePath();
                        this.routingTime = message.routingTime;
                    }
                    break;
                //所有元素的Id
                case "MapElementIds":
                    RENDERER.updateMapIndex(message.mapHash,
                        message.mapElementIds, message.mapRadius);
                    //                    console.log("your MapElementIds is : ");
                    //                    console.log(message.mapElementIds);
                    break;
                case "DefaultEndPoint":
                    STORE.routeEditingManager.updateDefaultRoutingEndPoint(message);
                    break;
                case "RoutePath":
                    RENDERER.updateRouting(message.routingTime, message.routePath);
                    //                    console.log("your routePath is: ");
                    //                    console.log(message);
                    //                    console.log(message.routePath);
                    break;
            }
        };
        this.websocket.onclose = event => {
            console.log("WebSocket connection closed, close_code: " + event.code);
            this.initialize();
        };

        // Request simulation world every 100ms.每100ms向后端请求RequestSimulationWorld类型数据
        clearInterval(this.timer);
        this.timer = setInterval(() => {
            if (this.websocket.readyState === this.websocket.OPEN) {
                // Load default routing end point.
                if (this.updatePOI) {
                    this.requestDefaultRoutingEndPoint();
                    this.updatePOI = false;
                }

                const requestPlanningData = STORE.options.showPNCMonitor;
                this.websocket.send(JSON.stringify({
                    //有请求，就会有响应，后端根据请求类型，调取相应的注册函数，发送相应的
                    //数据, planning的值为查看PNC面板
                    type: "RequestSimulationWorld",
                    planning: requestPlanningData,
                }));
            }
        }, this.simWorldUpdatePeriodMs);
    }

    updateMapIndex(message) {
        const now = new Date();
        const duration = now - this.mapLastUpdateTimestamp;
        if (message.mapHash && duration >= this.mapUpdatePeriodMs) {
            RENDERER.updateMapIndex(message.mapHash, message.mapElementIds, message.mapRadius);
            this.mapLastUpdateTimestamp = now;
        }
    }

    checkMessage(world) {
        const now = new Date().getTime();
        const duration = now - this.simWorldLastUpdateTimestamp;
        if (this.simWorldLastUpdateTimestamp !== 0 && duration > 200) {
            console.warn("Last sim_world_update took " + duration + "ms");
        }
        if (this.secondLastSeqNum === world.sequenceNum) {
            // Receiving multiple duplicated simulation_world messages
            // indicates a backend lag.
            console.warn("Received duplicate simulation_world:", this.lastSeqNum);
        }
        this.secondLastSeqNum = this.lastSeqNum;
        this.lastSeqNum = world.sequenceNum;
        this.simWorldLastUpdateTimestamp = now;
        //console.log("your world is: ");
        //console.log(world);
    }

    requestMapElementIdsByRadius(radius) {
        this.websocket.send(JSON.stringify({
            type: "RetrieveMapElementIdsByRadius",
            radius: radius,
        }));
    }

    requestRoute(start, waypoint, end, parking_status) {
        this.websocket.send(JSON.stringify({
            type: "SendRoutingRequest",
            start: start,
            end: end,
            waypoint: waypoint,
            parking_status: parking_status,
        }));
        //    console.log("your end is: ");
        //    console.log(end);
    }

    requestSummon(start, end) {
        this.websocket.send(JSON.stringify({
            type: "SendSummonRequest",
            start: start,
            end: end,
        }));
        //console.log("your waypoint is: ");
        //console.log(waypoint);
    }

    requestParking(start, end) {
        this.websocket.send(JSON.stringify({
            type: "SendParkingRequest",
            start: start,
            end: end,
        }));
        //console.log("your waypoint is: ");
        //console.log(waypoint);
    }

    requestParking_app(start, end, parking_id) {
        this.websocket.send(JSON.stringify({
            type: "app_SendParkingRequest",
            start: start,
            end: end,
            parking_id: parking_id,
        }));
        //console.log("your waypoint is: ");
        //console.log(waypoint);
    }

    requestParking_NOID(start) {
        this.websocket.send(JSON.stringify({
            type: "SendParking_NOIDRequest",
            start: start,
        }));
    }

    requestDefaultRoutingEndPoint() {
        this.websocket.send(JSON.stringify({
            type: "GetDefaultEndPoint",
        }));
    }

    resetBackend() {
        this.websocket.send(JSON.stringify({
            type: "Reset",
        }));
    }

    dumpMessages() {
        this.websocket.send(JSON.stringify({
            type: "Dump",
        }));
    }

    changeSetupMode(mode) {
        this.websocket.send(JSON.stringify({
            type: "ChangeMode",
            new_mode: mode,
        }));
    }

    changeMap(map) {
        this.websocket.send(JSON.stringify({
            type: "ChangeMap",
            new_map: map,
        }));
        this.updatePOI = true;
    }

    changeVehicle(vehcile) {
        this.websocket.send(JSON.stringify({
            type: "ChangeVehicle",
            new_vehicle: vehcile,
        }));
    }

    executeModeCommand(command) {
        this.websocket.send(JSON.stringify({
            type: "ExecuteModeCommand",
            command: command,
        }));
    }

    executeModuleCommand(module, command) {
        this.websocket.send(JSON.stringify({
            type: "ExecuteModuleCommand",
            module: module,
            command: command,
        }));
    }

    executeToolCommand(tool, command) {
        this.websocket.send(JSON.stringify({
            type: "ExecuteToolCommand",
            tool: tool,
            command: command,
        }));
    }

    changeDrivingMode(mode) {
        this.websocket.send(JSON.stringify({
            type: "ChangeDrivingMode",
            new_mode: mode,
        }));
        //        console.log("the mode is: ");
        //        console.log(mode);
    }

    changeDrivingAction(mode) {
        this.websocket.send(JSON.stringify({
            type: "changeDrivingAction",
            New_mode: mode,
        }));
    }

    submitDriveEvent(eventTimeMs, eventMessage) {
        this.websocket.send(JSON.stringify({
            type: "SubmitDriveEvent",
            event_time_ms: eventTimeMs,
            event_msg: eventMessage,
        }));
    }

    //sendVoicePiece(data) {
    //    this.websocket.send(JSON.stringify({
    //        type: "VoicePiece",
    //        data: btoa(String.fromCharCode(...data)),
    //    }));
    //}

    toggleSimControl(enable) {
        this.websocket.send(JSON.stringify({
            type: "ToggleSimControl",
            enable: enable,
        }));
    }

    requestRoutePath() {
        this.websocket.send(JSON.stringify({
            type: "RequestRoutePath",
        }));
    }

    //publishNavigationInfo(data) {
    //    this.websocket.send(data);
    //}
}
