import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { AlertTriangle, Droplets, Clock, User, Wifi, Battery } from 'lucide-react';

const InfusionMonitor = () => {
  const [patients, setPatients] = useState([]);

  // 从后端API获取数据
  const fetchPatients = async () => {
    try {
      const res = await axios.get('/api/patients');
      setPatients(res.data);
    } catch (err) {
      console.error('获取患者数据失败', err);
    }
  };

  useEffect(() => {
    fetchPatients(); // 首次加载
    const interval = setInterval(fetchPatients, 3000); // 每3秒刷新
    return () => clearInterval(interval);
  }, []);

  const getStatusColor = (status, remainingPercentage) => {
    if (remainingPercentage <= 5 && status === 'NORMAL') {
      return 'border-yellow-500 bg-white';
    }
    switch (status) {
      case 'NORMAL': return 'border-green-500 bg-white';
      case 'FAST_CONVERGENCE': return 'border-blue-500 bg-white';
      case 'INFUSION_ERROR': return 'border-red-500 bg-white';
      case 'INITIALIZING': return 'border-blue-500 bg-white';
      case 'INIT_ERROR': return 'border-red-500 bg-white';
      case 'COMPLETED': return 'border-red-500 bg-white';
      default: return 'border-gray-300 bg-white';
    }
  };

  const getStatusText = (status, remainingPercentage) => {
    if (remainingPercentage <= 5 && status === 'NORMAL') {
      return { text: '即将完成', color: 'text-yellow-800 bg-yellow-100' };
    }
    switch (status) {
      case 'NORMAL': return { text: '输液正常', color: 'text-green-800 bg-green-100' };
      case 'FAST_CONVERGENCE': return { text: '快速收敛', color: 'text-blue-800 bg-blue-100' };
      case 'INFUSION_ERROR': return { text: '输液异常', color: 'text-red-800 bg-red-100' };
      case 'INITIALIZING': return { text: '初始化中', color: 'text-blue-800 bg-blue-100' };
      case 'INIT_ERROR': return { text: '初始化异常', color: 'text-red-800 bg-red-100' };
      case 'COMPLETED': return { text: '输液完成', color: 'text-red-800 bg-red-100' };
      default: return { text: '未知', color: 'text-gray-800 bg-gray-100' };
    }
  };

  const PatientCard = ({ patient }) => {
    const percentage = (patient.remainingVolume / patient.totalVolume) * 100;
    const statusInfo = getStatusText(patient.systemState, percentage);

    return (
      <div className={`bg-white rounded-xl shadow-lg overflow-hidden border-l-8 ${getStatusColor(patient.systemState, percentage)} transition-all hover:shadow-2xl hover:-translate-y-1`}>
        <div className="p-6">
          {/* 头部信息 */}
          <div className="flex justify-between items-center mb-4">
            <div className="flex items-center gap-2">
              <User className="w-4 h-4 text-gray-600" />
              <span className="text-sm font-semibold text-gray-600">
                {patient.room}室 {patient.bed}床
              </span>
            </div>
            <div className="text-lg font-bold text-gray-800">{patient.name}</div>
          </div>

          <div className="flex gap-6">
            {/* 输液袋可视化 */}
            <div className="relative w-20 h-32 bg-gray-200 rounded-t-lg rounded-b-md flex flex-col justify-end">
              <div 
                className="absolute bottom-0 w-full bg-blue-400 rounded-b-md transition-all duration-1000"
                style={{ height: `${percentage}%` }}
              ></div>
              <div className="absolute -bottom-2 left-1/2 -translate-x-1/2 w-3 h-3 bg-gray-300 rounded-full"></div>
              <Droplets className="absolute top-2 left-1/2 -translate-x-1/2 w-4 h-4 text-blue-600" />
              <div className="absolute top-8 left-1/2 -translate-x-1/2 text-xs font-bold text-blue-800">
                {Math.round(percentage)}%
              </div>
              <div className="absolute top-12 left-1/2 -translate-x-1/2 text-xs text-blue-800">
                {patient.remainingVolume}/{patient.totalVolume}ml
              </div>
            </div>

            {/* 详细信息 */}
            <div className="flex-1 space-y-2">
              <p className="text-sm text-gray-500 font-medium">{patient.medication}</p>
              
              <div className="space-y-1">
                <p className="text-sm font-semibold text-gray-700 flex items-center gap-1">
                  <Droplets className="w-3 h-3 text-gray-500" />
                  滴速: <span className={patient.currentRate > 100 ? 'text-red-600 font-bold' : 'text-blue-600'}>
                    {patient.currentRate} 滴/分钟
                  </span>
                </p>
                
                <div className="flex items-center gap-1">
                  <Clock className="w-3 h-3 text-gray-500" />
                  <p className="text-sm font-semibold text-gray-700">
                    剩余: <span className={patient.estimatedTime < 10 ? 'text-yellow-600' : 'text-blue-600'}>
                      {patient.systemState === 'FAST_CONVERGENCE' ? '-' : `${patient.estimatedTime} 分钟`}
                    </span>
                  </p>
                </div>
              </div>
            </div>
          </div>
          
          {/* 底部状态和控制 */}
          <div className="mt-4 pt-4 border-t border-gray-200 flex justify-between items-center">
            <span className={`px-3 py-1 text-sm font-semibold rounded-full ${statusInfo.color}`}>
              {statusInfo.text}
            </span>
            
            <div className="flex items-center gap-2">
              <span className={`text-sm font-semibold py-1 px-3 rounded-lg ${
                patient.autoClamp === 1 
                  ? 'bg-red-500 text-white' 
                  : 'bg-teal-500 text-white'
              }`}>
                {patient.autoClamp === 1 ? '夹断' : '通液'}
              </span>
            </div>
          </div>

          {/* 设备状态指示器 */}
          <div className="mt-3 flex justify-between text-xs text-gray-500">
            <div className="flex items-center gap-1">
              <Wifi className="w-3 h-3 text-green-500" />
              <span>WiFi 连接正常</span>
              <span className="ml-2 flex items-center gap-1">
                <svg className="w-3 h-3 text-green-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z" />
                </svg>
                ID: {patient.deviceId || '未分配'}
              </span>
            </div>
            <div className="flex items-center gap-1">
              <Battery className="w-3 h-3 text-green-500" />
              <span>电量 85%</span>
            </div>
            <span>更新: {patient.lastUpdate ? (new Date(patient.lastUpdate)).toLocaleTimeString() : ''}</span>
          </div>
        </div>
      </div>
    );
  };

  const activeCount = patients.filter(p => p.systemState === 'NORMAL').length;
  const warningCount = patients.filter(p => (p.remainingVolume / p.totalVolume) * 100 <= 5).length;
  const alertCount = patients.filter(p => p.systemState === 'INFUSION_ERROR' || p.systemState === 'COMPLETED').length;

  return (
    <div className="min-h-screen bg-gray-100">
      <div className="container mx-auto p-4 md:p-8">
        {/* 头部 */}
        <header className="text-center mb-8">
          <h1 className="text-3xl font-bold text-gray-800 mb-2">智能输液监控系统</h1>
          <p className="text-gray-500 mb-4">实时监控 · 智能预警 · 自动控制</p>
          
          {/* 统计信息 */}
          <div className="flex justify-center gap-6 text-sm">
            <div className="flex items-center gap-1">
              <div className="w-3 h-3 bg-green-500 rounded-full"></div>
              <span>正常: {activeCount}</span>
            </div>
            <div className="flex items-center gap-1">
              <div className="w-3 h-3 bg-yellow-500 rounded-full"></div>
              <span>即将完成: {warningCount}</span>
            </div>
            <div className="flex items-center gap-1">
              <div className="w-3 h-3 bg-red-500 rounded-full animate-pulse"></div>
              <span>需要关注: {alertCount}</span>
            </div>
          </div>
        </header>

        {/* 患者卡片网格 */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-6">
          {patients.map(patient => (
            <PatientCard key={patient.id} patient={patient} />
          ))}
        </div>

        {/* 紧急警报提示 */}
        {alertCount > 0 && (
          <div className="fixed bottom-4 right-4 bg-red-500 text-white p-4 rounded-lg shadow-lg animate-bounce">
            <div className="flex items-center gap-2">
              <AlertTriangle className="w-5 h-5" />
              <span className="font-semibold">有 {alertCount} 个病床需要立即关注！</span>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default InfusionMonitor;
