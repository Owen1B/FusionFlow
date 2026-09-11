# 作者与依赖

作者：郑皓文（Owen）。项目内容涵盖重量与滴速估计、单滴质量标定、融合模块、嵌入式集成及监测界面。

## 依赖入口

设备依赖以 [`platformio.ini`](../platformio.ini) 为准，包括 Arduino、HX711、Adafruit 显示/NeoPixel 库、WebSockets、U8g2 和 ArduinoJson。前端依赖以 `server/frontend/package.json` 与对应 lockfile 为准，包括 React、Axios、Lucide 和页面构建工具。各依赖保留原有许可与署名。

项目根目录许可见 [LICENSE](../LICENSE)。子目录若有单独声明，使用时一并检查；例如 `server/package.json` 中保留了独立的包元数据。

## 演示材料

README 复用仓库中的界面截图、实验图和既有演示视频入口。图表描述项目已有记录，后续补充量化结论时同时提供数据来源、参数和参考测量。展示数据用于界面与台架实验说明。
