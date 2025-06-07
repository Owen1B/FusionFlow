# 智能输液监控系统 API 文档

## 基础信息
- 基础URL: `http://localhost:5000`
- 所有请求和响应均使用 JSON 格式
- 时间格式：ISO 8601 (例如：`2024-03-06T17:03:52.168Z`)

## 系统状态说明

### 输液状态 (systemState)
- `NORMAL`: 输液正常
- `FAST_CONVERGENCE`: 快速收敛
- `INFUSION_ERROR`: 输液异常
- `INITIALIZING`: 初始化中
- `INIT_ERROR`: 初始化异常
- `COMPLETED`: 已完成

### 智能控制状态 (autoClamp)
- `1`: 夹断
- `0`: 通水

## API 端点

### 1. 获取所有患者数据
```http
GET /api/patients
```

响应示例：
```json
[
    {
        "id": 1,
        "deviceId": "101",
        "name": "刘洋",
        "room": "302",
        "bed": "04",
        "medication": "5% 葡萄糖注射液",
        "totalVolume": 500,
        "remainingVolume": 375,
        "currentRate": 60,
        "estimatedTime": 45,
        "systemState": "NORMAL",
        "autoClamp": true,
        "lastUpdate": "2024-03-06T17:03:52.168Z",
        "systemStateText": "输液正常"
    }
]
```

### 2. 更新或新增患者数据
```http
POST /api/patients
```

请求体示例：
```json
{
    "deviceId": "101",
    "totalVolume": 500,
    "remainingVolume": 375,
    "currentRate": 60,
    "estimatedTime": 45,
    "systemState": "NORMAL"
}
```

响应：返回更新后的完整患者数据

### 3. 更新患者基本信息
```http
PUT /api/patients/:deviceId
```

请求体示例：
```json
{
    "name": "张三",
    "room": "301",
    "bed": "01",
    "medication": "0.9% 氯化钠注射液",
    "autoClamp": true
}
```

响应：返回更新后的完整患者数据

### 4. 删除患者
```http
DELETE /api/patients/:deviceId
```

响应：
```json
{
    "success": true
}
```

## 数据字段说明

### 患者数据字段
- `id`: 数据库自增ID
- `deviceId`: 设备ID（唯一标识）
- `name`: 患者姓名
- `room`: 病房号
- `bed`: 床位号
- `medication`: 药品名称
- `totalVolume`: 总输液量（ml）
- `remainingVolume`: 剩余输液量（ml）
- `currentRate`: 当前滴速（滴/分钟）
- `estimatedTime`: 预计剩余时间（分钟）
- `systemState`: 系统状态
- `autoClamp`: 智能控制状态
- `lastUpdate`: 最后更新时间
- `systemStateText`: 系统状态中文描述

## 错误处理
所有错误响应将返回适当的 HTTP 状态码和错误信息：
```json
{
    "error": "错误描述信息"
}
```

## 注意事项
1. 所有时间戳使用 UTC 时间
2. 设备ID 必须唯一
3. 输液量单位为毫升（ml）
4. 滴速单位为滴/分钟
5. 预计剩余时间单位为分钟 
 