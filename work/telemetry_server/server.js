const http = require("http");
const { WebSocketServer } = require("ws");

const PORT = Number(process.env.PORT || 8000);

let esp32 = null;
let latestTelemetry = null;
let requestSeq = 0;

function sendJson(ws, obj) {
  if (!ws || ws.readyState !== ws.OPEN) return false;
  ws.send(JSON.stringify(obj));
  return true;
}

const server = http.createServer((req, res) => {
  if (req.url === "/latest") {
    res.writeHead(200, {
      "content-type": "application/json; charset=utf-8",
      "access-control-allow-origin": "*",
    });
    res.end(JSON.stringify({
      ok: true,
      esp32_online: !!esp32,
      latest: latestTelemetry,
    }));
    return;
  }

  res.writeHead(200, { "content-type": "text/plain; charset=utf-8" });
  res.end("xiaoan telemetry relay\nGET /latest\nWS /ws/esp32\n");
});

const wss = new WebSocketServer({ server });

wss.on("connection", (ws, req) => {
  const path = req.url || "/";
  console.log(`[ws] connected ${path}`);

  if (path === "/ws/esp32") {
    esp32 = ws;
    ws.on("close", () => {
      if (esp32 === ws) esp32 = null;
      console.log("[esp32] disconnected");
    });
  }

  ws.on("message", (buf, isBinary) => {
    if (isBinary) return;
    let msg;
    try {
      msg = JSON.parse(buf.toString("utf8"));
    } catch {
      console.log("[ws] non-json:", buf.toString("utf8"));
      return;
    }

    if (msg.type === "hello") {
      console.log("[esp32] hello:", msg);
      return;
    }

    if (msg.type === "sensor_data") {
      latestTelemetry = {
        received_at: new Date().toISOString(),
        data: msg.data,
      };
      console.log("[sensor_data]", JSON.stringify(latestTelemetry, null, 2));
      return;
    }

    if (msg.type !== "ping") {
      console.log("[ws]", msg);
    }
  });
});

setInterval(() => {
  if (!esp32 || esp32.readyState !== esp32.OPEN) return;
  sendJson(esp32, {
    type: "read_sensors",
    request_id: `pc-${Date.now()}-${++requestSeq}`,
  });
}, 2000);

server.listen(PORT, "0.0.0.0", () => {
  console.log(`xiaoan telemetry relay listening on http://0.0.0.0:${PORT}`);
  console.log(`ESP32 WS URI: ws://<YOUR_PC_IP>:${PORT}/ws/esp32`);
  console.log(`Latest telemetry: http://<YOUR_PC_IP>:${PORT}/latest`);
});
