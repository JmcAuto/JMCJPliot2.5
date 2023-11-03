//webWorker主要作用是创建一个子线程，接收主线程消息，并对消息进行解码，
//转换成字符串形式，并
//处理后的数据发送给主线程,如果是字符串，转换为json格式；如果是其他，
//则进行解码转换为对应的
//数据结构
const protobuf = require("protobufjs/light");
const simWorldRoot = protobuf.Root.fromJSON(
    require("proto_bundle/sim_world_proto_bundle.json")
);
const SimWorldMessage = simWorldRoot.lookupType("jmc_auto.dreamview.SimulationWorld");
const mapMessage = simWorldRoot.lookupType("jmc_auto.hdmap.Map");
//const pointCloudRoot = protobuf.Root.fromJSON(
//    require("proto_bundle/point_cloud_proto_bundle.json")
//);
//const pointCloudMessage = pointCloudRoot.lookupType("jmc_auto.dreamview.PointCloud");
//子线程给主线程发送消息
self.addEventListener("message", event => {
    let message = null;
    const data = event.data.data;
    switch (event.data.source) {
        case "realtime":
            if (typeof data === "string") {
                message = JSON.parse(data);
                //parse()参数为字符串格式，转换为JS对象格式
            } else {
                message = SimWorldMessage.toObject(
                    SimWorldMessage.decode(new Uint8Array(data)),
                    //将二进制字符串数组转化为对象，数据结构为{枚举： 字符串}。
                    { enums: String });
                message.type = "SimWorldUpdate";

            }
            break;
        case "map":
            message = mapMessage.toObject(
                mapMessage.decode(new Uint8Array(data)),
                {enums: String});
            message.type = "MapData";
            break;
/*        case "point_cloud":
            if (typeof data === "string") {
                message = JSON.parse(data);
            } else {
                message = pointCloudMessage.toObject(
                    pointCloudMessage.decode(new Uint8Array(data)), {arrays: true});
            }
            break;
*/
    }

    if (message) {
        self.postMessage(message);
    }
});
