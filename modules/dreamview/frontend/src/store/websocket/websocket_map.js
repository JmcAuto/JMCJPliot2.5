import STORE from "store";
import RENDERER from "renderer";
import Worker from 'utils/webworker.js';

export default class MapDataWebSocketEndpoint {
    constructor(serverAddr) {
        this.serverAddr = serverAddr;
        this.websocket = null;
        this.currentMode = null;
        this.worker = new Worker();
    }

    initialize() {
        try {
            this.websocket = new WebSocket(this.serverAddr);
            this.websocket.binaryType = "arraybuffer";
        } catch (error) {
            console.error("Failed to establish a connection: " + error);
            setTimeout(() => {
                this.initialize();
            }, 1000);
            return;
        }
        this.websocket.onmessage = event => {
//            console.log("your map arraybuffer eventdata is: ");
//            console.log(event.data);
            this.worker.postMessage({
                source: 'map',
                data: event.data,
            });
        };
        this.worker.onmessage = event => {
//            console.log("your first true map1 eventdata is: ");
//            console.log(event.data);
            const removeOldMap =
                STORE.hmi.inNavigationMode || this.currentMode !== STORE.hmi.currentMode;
            this.currentMode = STORE.hmi.currentMode;
            RENDERER.updateMap(event.data, removeOldMap);
//            console.log("your true map1 eventdata is: ");
//            console.log(event.data);
            /*const storage = window.localStorage;
            const trueMap1Data = JSON.stringify(event.data);
            storage.setItem("mapData", trueMap1Data);*/
            STORE.setInitializationStatus(true);
        };
        this.websocket.onclose = event => {
            console.log("WebSocket connection closed with code: " + event.code);
            this.initialize();
        };
    }

    requestMapData(elements) {
        this.websocket.send(JSON.stringify({
            type: "RetrieveMapData",
            elements: elements,
        }));
        //console.log("your map2 eventdata elements is: ");
        //console.log(elements);
    }

    requestRelativeMapData(elements) {
        this.websocket.send(JSON.stringify({
            type: "RetrieveRelativeMapData",
            elements: elements,
        }));
        //console.log("your map3 eventdata relative elements is: ");
        //console.log(elements);
    }
}
