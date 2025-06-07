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
        body { 
            font-family: Arial, sans-serif; 
            margin: 0;
            padding: 20px;
            background-color: #f0f2f5;
            color: #333;
        }
        .container { 
            max-width: 1200px;
            margin: 0 auto;
            background-color: #fff;
            padding: 20px;
            border-radius: 12px;
            box-shadow: 0 2px 12px rgba(0,0,0,0.1);
        }
        h1 { 
            color: #1a73e8;
            text-align: center;
            margin-bottom: 20px;
            font-size: 24px;
        }
        .status-container {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 20px;
            padding: 10px;
            background-color: #f8f9fa;
            border-radius: 8px;
        }
        #wsState {
            font-style: italic;
            color: #666;
        }
        #systemState {
            padding: 8px 16px;
            border-radius: 6px;
            font-weight: bold;
            color: white;
            text-align: center;
            min-width: 120px;
        }
        .data-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-top: 20px;
        }
        .data-section {
            background-color: #fff;
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1);
        }
        .section-title {
            color: #1a73e8;
            font-size: 18px;
            margin-bottom: 15px;
            padding-bottom: 8px;
            border-bottom: 2px solid #e8f0fe;
        }
        .data-row {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #f0f0f0;
        }
        .data-row:last-child {
            border-bottom: none;
        }
        .label {
            color: #666;
            font-weight: 500;
        }
        .value {
            color: #1a73e8;
            font-weight: 600;
        }
        .progress-container {
            margin-top: 20px;
            padding: 15px;
            background-color: #f8f9fa;
            border-radius: 8px;
        }
        .progress-bar {
            width: 100%;
            height: 20px;
            background-color: #e8f0fe;
            border-radius: 10px;
            overflow: hidden;
            margin-top: 10px;
        }
        .progress-fill {
            height: 100%;
            background-color: #1a73e8;
            transition: width 0.3s ease;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>智能输液监控系统</h1>
        
        <div class="status-container">
            <div id="wsState">正在连接WebSocket...</div>
            <div id="systemState"></div>
        </div>

        <div class="data-grid">
            <div class="data-section">
                <div class="section-title">重量传感器数据</div>
                <div class="data-row">
                    <span class="label">原始重量:</span>
                    <span class="value" id="raw_weight_g">-</span>
                </div>
                <div class="data-row">
                    <span class="label">滤波后重量:</span>
                    <span class="value" id="filt_weight_g">-</span>
                </div>
                <div class="data-row">
                    <span class="label">重量流速:</span>
                    <span class="value" id="flow_weight_gps">- g/s</span>
                </div>
            </div>

            <div class="data-section">
                <div class="section-title">滴速传感器数据</div>
                <div class="data-row">
                    <span class="label">本周期滴数:</span>
                    <span class="value" id="drops_period">-</span>
                </div>
                <div class="data-row">
                    <span class="label">原始滴速:</span>
                    <span class="value" id="raw_drip_rate_dps">- dps</span>
                </div>
                <div class="data-row">
                    <span class="label">滤波后滴速:</span>
                    <span class="value" id="filt_drip_rate_dps">- dps</span>
                </div>
                <div class="data-row">
                    <span class="label">每滴重量:</span>
                    <span class="value" id="wpd_g">- g/drip</span>
                </div>
                <div class="data-row">
                    <span class="label">WPD校准中:</span>
                    <span class="value" id="wpd_calibrating">-</span>
                </div>
            </div>

            <div class="data-section">
                <div class="section-title">融合数据</div>
                <div class="data-row">
                    <span class="label">融合后流速:</span>
                    <span class="value" id="fused_flow_gps">- g/s</span>
                </div>
                <div class="data-row">
                    <span class="label">融合后剩余重量:</span>
                    <span class="value" id="fused_rem_w_g">- g</span>
                </div>
                <div class="data-row">
                    <span class="label">预计剩余时间:</span>
                    <span class="value" id="remaining_time_s">-</span>
                </div>
            </div>
        </div>

        <div class="progress-container">
            <div class="section-title">输液进度</div>
            <div class="data-row">
                <span class="label">完成百分比:</span>
                <span class="value" id="infusion_progress_percent">-</span>
            </div>
            <div class="progress-bar">
                <div class="progress-fill" id="progress_fill" style="width: 0%"></div>
            </div>
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
                    console.log("Received ESP32 Alert: " + messageText);
                    return;
                }
                if (messageText.startsWith("INITIAL_PARAMS:")) {
                    const params = messageText.split(':')[1].split(',');
                    systemInitialWeight = parseFloat(params[0]);
                    targetEmptyWeight = parseFloat(params[1]);
                    console.log("Received initial params: Initial Weight=" + systemInitialWeight + "g, Target Empty=" + targetEmptyWeight + "g");
                    return;
                }

                const data = messageText.split(',');
                // Expecting 26 columns now (added system state)
                if (data.length >= 26) {
                    if (data[0].toLowerCase().includes("timestamp_ms")) return; 
                    
                    // 更新系统状态显示
                    const systemState = parseInt(data[25]);
                    let stateText = "";
                    let stateColor = "";
                    switch(systemState) {
                        case 0: // INITIALIZING
                            stateText = "系统初始化中";
                            stateColor = "#FFA500"; // 橙色
                            break;
                        case 1: // INIT_ERROR
                            stateText = "系统初始化异常";
                            stateColor = "#FF0000"; // 红色
                            break;
                        case 2: // NORMAL
                            stateText = "正常输液";
                            stateColor = "#00FF00"; // 绿色
                            break;
                        case 3: // INFUSION_ERROR
                            stateText = "输液异常";
                            stateColor = "#FF0000"; // 红色
                            break;
                        case 4: // FAST_CONVERGENCE
                            stateText = "快速收敛模式";
                            stateColor = "#0000FF"; // 蓝色
                            break;
                        default:
                            stateText = "未知状态";
                            stateColor = "#808080"; // 灰色
                    }
                    document.getElementById("systemState").textContent = stateText;
                    document.getElementById("systemState").style.backgroundColor = stateColor;
                    document.getElementById("systemState").style.color = "#FFFFFF";
                    
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

                // 更新所有数据显示
                document.getElementById("raw_weight_g").textContent = lastData.raw_weight_g + " g";
                document.getElementById("filt_weight_g").textContent = lastData.filt_weight_g + " g";
                document.getElementById("flow_weight_gps").textContent = lastData.flow_weight_gps + " g/s";
                document.getElementById("drops_period").textContent = lastData.drops_period;
                document.getElementById("raw_drip_rate_dps").textContent = lastData.raw_drip_rate_dps + " dps";
                document.getElementById("filt_drip_rate_dps").textContent = lastData.filt_drip_rate_dps + " dps";
                document.getElementById("wpd_g").textContent = lastData.wpd_g + " g/drip";
                document.getElementById("wpd_calibrating").textContent = lastData.wpd_calibrating;
                document.getElementById("fused_flow_gps").textContent = lastData.fused_flow_gps + " g/s";
                document.getElementById("fused_rem_w_g").textContent = lastData.fused_rem_w_g + " g";

                // 更新剩余时间显示
                let rt_s_val = parseFloat(lastData.remaining_time_s);
                let rt_min_str = "---";
                if (!isNaN(rt_s_val)) {
                    if (rt_s_val > 0 && rt_s_val < (3600*99)) {
                        rt_min_str = Math.floor(rt_s_val / 60).toString();
                    } else {
                        let current_fused_rem_w = parseFloat(lastData.fused_rem_w_g);
                        let current_fused_flow = parseFloat(lastData.fused_flow_gps);
                        if (!isNaN(current_fused_rem_w) && !isNaN(targetEmptyWeight) && !isNaN(current_fused_flow) &&
                            current_fused_rem_w <= targetEmptyWeight + 1.0 && Math.abs(current_fused_flow) < 0.001) {
                            rt_min_str = "0";
                        } else if (rt_s_val == 0) {
                            rt_min_str = "0";
                        }
                    }
                    document.getElementById("remaining_time_s").textContent = rt_s_val.toFixed(0) + " 秒 (" + rt_min_str + " 分)";
                } else {
                    document.getElementById("remaining_time_s").textContent = "-";
                }

                // 更新进度显示
                if (lastData.infusion_progress_percent && lastData.infusion_progress_percent !== "-") {
                    const progress = lastData.infusion_progress_percent;
                    document.getElementById("infusion_progress_percent").textContent = progress + "%";
                    document.getElementById("progress_fill").style.width = progress + "%";
                } else {
                    document.getElementById("infusion_progress_percent").textContent = "-";
                    document.getElementById("progress_fill").style.width = "0%";
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