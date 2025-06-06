#ifndef WEBPAGE_H
#define WEBPAGE_H

// HTML页面内容
const char HTML_WEBPAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>智能输液监控</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 10px; background-color: #f4f4f4; color: #333; }
        .container { background-color: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
        h1 { color: #0056b3; text-align: center; }
        table { width: 100%; border-collapse: collapse; margin-top: 15px; }
        th, td { text-align: left; padding: 8px; border-bottom: 1px solid #ddd; }
        th { background-color: #007bff; color: white; }
        .label { font-weight: bold; color: #555; }
        .value { color: #000; }
        .button-container { margin-top: 20px; text-align: center; }
        button { background-color: #007bff; color: white; padding: 10px 15px; border: none; border-radius: 5px; cursor: pointer; font-size: 1em; margin: 5px; }
        button:hover { background-color: #0056b3; }
        #wsState { margin-top:10px; font-style:italic; text-align: center; }
    </style>
</head>
<body>
    <div class="container">
        <h1>智能输液监控系统</h1>
        
        <div id="wsState">正在连接WebSocket...</div>

        <table>
            <tr><td class="label">系统时间戳 (ms):</td><td class="value" id="timestamp_ms">-</td></tr>
            <tr><th colspan="2">重量传感器 (Weight Sensor)</th></tr>
            <tr><td class="label">原始重量 (g):</td><td class="value" id="raw_weight_g">-</td></tr>
            <tr><td class="label">滤波后重量 (g):</td><td class="value" id="filt_weight_g">-</td></tr>
            <tr><td class="label">重量流速 (g/s):</td><td class="value" id="flow_weight_gps">-</td></tr>
            <tr><th colspan="2">滴速传感器 (Drip Sensor)</th></tr>
            <tr><td class="label">本周期滴数:</td><td class="value" id="drops_period">-</td></tr>
            <tr><td class="label">原始滴速 (dps):</td><td class="value" id="raw_drip_rate_dps">-</td></tr>
            <tr><td class="label">滤波后滴速 (dps):</td><td class="value" id="filt_drip_rate_dps">-</td></tr>
            <tr><td class="label">每滴重量 (g/drip):</td><td class="value" id="wpd_g">-</td></tr>
            <tr><td class="label">WPD校准中:</td><td class="value" id="wpd_calibrating">-</td></tr>
            <tr><td class="label">剩余重量 (滴数法) (g):</td><td class="value" id="rem_w_drip_calc_g">-</td></tr>
            <tr><td class="label">滴速流速 (g/s):</td><td class="value" id="flow_drip_gps">-</td></tr>
            <tr><td class="label">累计总滴数:</td><td class="value" id="total_drops_count">-</td></tr>
            <tr><td class="label">滴速传感器初始重量 (g):</td><td class="value" id="drip_initial_weight_g">-</td></tr>
            <tr><th colspan="2">融合数据 & 预测 (Fused Data & Prediction)</th></tr>
            <tr><td class="label">融合后流速 (g/s):</td><td class="value" id="fused_flow_gps">-</td></tr>
            <tr><td class="label">融合后剩余重量 (g):</td><td class="value" id="fused_rem_w_g">-</td></tr>
            <tr><td class="label">预计剩余时间:</td><td class="value" id="remaining_time_s">- 秒</td></tr>
            <tr><td class="label">输液进度 (%):</td><td class="value" id="infusion_progress_percent">-</td></tr>
        </table>

        <div class="button-container">
            <button onclick="sendWSCmd('CALIBRATE_WPD_START')">开始WPD校准</button>
            <button onclick="sendWSCmd('CALIBRATE_WPD_STOP')">停止WPD校准</button>
        </div>
    </div>

    <script>
        let ws;
        let lastData = {};
        const displayInterval = 1000; // 页面数据更新间隔 (ms)，例如1秒
        let displayTimer;
        let systemInitialWeight = 0; // Still needed for context if progress is not available from WS
        let targetEmptyWeight = 60.0; // Still needed for context if progress is not available from WS

        function connectWebSocket() {
            const wsUrl = "ws://" + window.location.hostname + ":81/";
            ws = new WebSocket(wsUrl);
            document.getElementById("wsState").textContent = "正在连接到: " + wsUrl;

            ws.onopen = function(event) {
                document.getElementById("wsState").textContent = "WebSocket 已连接";
                console.log("WebSocket connection opened");
                if(displayTimer) clearInterval(displayTimer);
                displayTimer = setInterval(updateDisplay, displayInterval);
                 // Request initial parameters or wait for them
            };

            ws.onmessage = function(event) {
                const messageText = event.data;
                if (messageText.startsWith("ALERT:")) {
                    alert(messageText);
                    console.log("Received ESP32 Alert: " + messageText);
                    return;
                }
                if (messageText.startsWith("INITIAL_PARAMS:")) {
                    const params = messageText.split(':')[1].split(',');
                    systemInitialWeight = parseFloat(params[0]); // Keep these updated for any fallback or other JS logic
                    targetEmptyWeight = parseFloat(params[1]);
                    console.log("Received initial params: Initial Weight=" + systemInitialWeight + "g, Target Empty=" + targetEmptyWeight + "g");
                    return;
                }

                const data = messageText.split(',');
                // Expecting 25 columns now
                // 0:timestamp_ms, 1:raw_w, 2:filt_w, 3:raw_flow_w_gps, 4:filt_flow_w_gps,
                // 5:drops_p, 6:raw_dps, 7:filt_dps, 8:raw_flow_d_gps, 9:filt_flow_d_gps,
                // 10:wpd, 11:wpd_cal_status, 12:wpd_l_cal_active, 
                // 13:rem_w_drip_calc_g,
                // 14:fused_gps, 15:fused_rem_w_g,
                // 16:rem_t_raw_w, 17:rem_t_filt_w, 18:rem_t_raw_d, 19:rem_t_filt_d, 
                // 20:rem_t_fused (primary remaining time in seconds)
                // 21:drip_total_drops
                // 22:drip_initial_weight
                // 23:wpd_cumulative
                // 24:esp_calculated_progress_percent (-1 if not set)
                if (data.length >= 25) { // Check for 25 data points
                    if (data[0].toLowerCase().includes("timestamp_ms")) return; 
                    
                    lastData.timestamp_ms = data[0];
                    lastData.raw_weight_g = parseFloat(data[1]).toFixed(2);
                    lastData.filt_weight_g = parseFloat(data[2]).toFixed(2);
                    lastData.raw_flow_weight_gps = parseFloat(data[3]).toFixed(4); 
                    lastData.flow_weight_gps = parseFloat(data[4]).toFixed(4); 
                    lastData.drops_period = data[5];
                    lastData.raw_drip_rate_dps = parseFloat(data[6]).toFixed(2);
                    lastData.filt_drip_rate_dps = parseFloat(data[7]).toFixed(2);
                    lastData.raw_flow_drip_gps = parseFloat(data[8]).toFixed(4); 
                    lastData.flow_drip_gps = parseFloat(data[9]).toFixed(4); 
                    lastData.wpd_g = parseFloat(data[10]).toFixed(4);
                    lastData.wpd_calibrating = (data[11] === "1" || data[12] === "1") ? "是" : "否"; // Combine WPD cal status
                    // lastData.wpd_long_calibrating = (data[12] === "1") ? "是" : "否"; // Not displayed separately currently
                    lastData.rem_w_drip_calc_g = parseFloat(data[13]).toFixed(2);
                    lastData.fused_flow_gps = parseFloat(data[14]).toFixed(4); 
                    lastData.fused_rem_w_g = parseFloat(data[15]).toFixed(2);
                    lastData.remaining_time_s = parseFloat(data[20]).toFixed(0); 
                    
                    lastData.drip_total_drops = data[21];
                    lastData.drip_initial_weight_g = parseFloat(data[22]).toFixed(2);
                    // lastData.wpd_cumulative = parseFloat(data[23]).toFixed(5); // Not displayed currently

                    // Use progress directly from ESP32
                    let esp_progress = parseFloat(data[24]);
                    if (esp_progress >= 0 && esp_progress <= 100) {
                        lastData.infusion_progress_percent = esp_progress.toFixed(1);
                    } else {
                        lastData.infusion_progress_percent = "-"; // Show "-" if ESP sends -1 or invalid
                    }
                    // Removed old JS-side progress calculation
                } else {
                    // console.log("Received incomplete data (expected 25 fields): " + event.data);
                }
            };

            function updateDisplay() {
                 if (!lastData.timestamp_ms) return; 

                 document.getElementById("timestamp_ms").textContent = lastData.timestamp_ms;
                 document.getElementById("raw_weight_g").textContent = lastData.raw_weight_g;
                 document.getElementById("filt_weight_g").textContent = lastData.filt_weight_g;
                 document.getElementById("flow_weight_gps").textContent = lastData.flow_weight_gps; 
                 document.getElementById("drops_period").textContent = lastData.drops_period;
                 document.getElementById("raw_drip_rate_dps").textContent = lastData.raw_drip_rate_dps;
                 document.getElementById("filt_drip_rate_dps").textContent = lastData.filt_drip_rate_dps;
                 document.getElementById("wpd_g").textContent = lastData.wpd_g;
                 document.getElementById("wpd_calibrating").textContent = lastData.wpd_calibrating; 
                 document.getElementById("flow_drip_gps").textContent = lastData.flow_drip_gps; 
                 document.getElementById("rem_w_drip_calc_g").textContent = lastData.rem_w_drip_calc_g;
                 
                 document.getElementById("total_drops_count").textContent = lastData.drip_total_drops || "-";
                 document.getElementById("drip_initial_weight_g").textContent = lastData.drip_initial_weight_g || "-";

                 document.getElementById("fused_flow_gps").textContent = lastData.fused_flow_gps;
                 document.getElementById("fused_rem_w_g").textContent = lastData.fused_rem_w_g;
                 
                 // Update remaining time display
                 let rt_s_val = parseFloat(lastData.remaining_time_s);
                 let rt_min_str = "---";
                 if (!isNaN(rt_s_val)) {
                     if (rt_s_val > 0 && rt_s_val < (3600*99)) {
                         rt_min_str = Math.floor(rt_s_val / 60).toString();
                     } else {
                         // Check for completion/near empty and low flow for 0 min display
                         // Need fused_rem_w_g (already in lastData), targetEmptyWeight (global JS), fused_flow_gps (already in lastData)
                         let current_fused_rem_w = parseFloat(lastData.fused_rem_w_g);
                         let current_fused_flow = parseFloat(lastData.fused_flow_gps);
                         if (!isNaN(current_fused_rem_w) && !isNaN(targetEmptyWeight) && !isNaN(current_fused_flow) &&
                             current_fused_rem_w <= targetEmptyWeight + 1.0 && Math.abs(current_fused_flow) < 0.001) {
                             rt_min_str = "0";
                         } else if (rt_s_val == 0) { // If remaining time is exactly 0 seconds, minutes should be 0.
                            rt_min_str = "0";
                         }
                     }
                     document.getElementById("remaining_time_s").textContent = rt_s_val.toFixed(0) + " 秒 (" + rt_min_str + " 分)";
                 } else {
                     document.getElementById("remaining_time_s").textContent = "- 秒 (--- 分)";
                 }

                 // Update progress display
                 if (lastData.infusion_progress_percent && lastData.infusion_progress_percent !== "-") {
                     document.getElementById("infusion_progress_percent").textContent = lastData.infusion_progress_percent + "%";
                 } else {
                     document.getElementById("infusion_progress_percent").textContent = "-%";
                 }
            }

            ws.onerror = function(error) {
                document.getElementById("wsState").textContent = "WebSocket 错误!";
                console.error("WebSocket Error: ", error);
            };

            ws.onclose = function(event) {
                document.getElementById("wsState").textContent = "WebSocket 已断开. 正在尝试重连...";
                console.log("WebSocket connection closed. Code: " + event.code + ", Reason: " + event.reason);
                if(displayTimer) clearInterval(displayTimer);
                // 尝试5秒后重连
                setTimeout(connectWebSocket, 5000);
            };
        }

        function sendWSCmd(cmd) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(cmd);
                console.log("Sent WS command: " + cmd);
            } else {
                console.log("WebSocket not connected. Cannot send command: " + cmd);
                document.getElementById("wsState").textContent = "WebSocket未连接，无法发送命令!";
            }
        }

        // 页面加载完成后开始连接WebSocket
        window.onload = connectWebSocket;
    </script>
</body>
</html>
)rawliteral";

#endif // WEBPAGE_H 