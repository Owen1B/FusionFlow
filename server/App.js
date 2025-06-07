import React, { useState, useEffect } from 'react';
import { AlertTriangle, Droplets, Clock, User, Wifi, Battery } from 'lucide-react';

const InfusionMonitor = () => {
  const [patients, setPatients] = useState([
    {
      id: 1,
      name: '王小明',
      room: '302',
      bed: '01',
      medication: '0.9% 氯化钠注射液',
      totalVolume: 500,
      remainingVolume: 375,
      currentRate: 60,
      estimatedTime: 45,
      status: 'active',
      autoClamp: true,
      lastUpdate: new Date()
    },
    {
      id: 2,
      name: '李思思',
      room: '302',
      bed: '02',
      medication: '5% 葡萄糖注射液',
      totalVolume: 250,
      remainingVolume: 40,
      currentRate: 55,
      estimatedTime: 8,
      status: 'warning',
      autoClamp: true,
      lastUpdate: new Date()
    },
    {
      id: 3,
      name: '张伟',
      room: '303',
      bed: '01',
      medication: '甲硝唑注射液',
      totalVolume: 100,
      remainingVolume: 0,
      currentRate: 0,
      estimatedTime: 0,
      status: 'completed',
      autoClamp: true,
      lastUpdate: new Date()
    },
    {
      id: 4,
      name: '赵静',
      room: '305',
      bed: '03',
      medication: '青霉素 (需皮试)',
      totalVolume: 250,
      remainingVolume: 125,
      currentRate: 120,
      estimatedTime: 15,
      status: 'alert',
      autoClamp: true,
      lastUpdate: new Date()
    }
  ]);

  // 模拟实时数据更新
  useEffect(() => {
    const interval = setInterval(() => {
      setPatients(prevPatients => 
        prevPatients.map(patient => {
          if (patient.status === 'active' || patient.status === 'warning') {
            const newRemaining = Math.max(0, patient.remainingVolume - Math.random() * 5);
            const newTime = newRemaining > 0 ? Math.ceil(newRemaining / patient.currentRate * 60) : 0;
            
            let newStatus = patient.status;
            if (newRemaining === 0) {
              newStatus = 'completed';
            } else if (newRemaining < patient.totalVolume * 0.2) {
              newStatus = 'warning';
            }
            
            return {
              ...patient,
              remainingVolume: parseFloat(newRemaining.toFixed(1)),
              estimatedTime: newTime,
              status: newStatus,
              lastUpdate: new Date()
            };
          }
          return patient;
        })
      );
    }, 5000);

    return () => clearInterval(interval);
  }, []);

  const getStatusColor = (status) => {
    switch (status) {
      case 'active': return 'border-green-500 bg-green-50';
      case 'warning': return 'border-yellow-500 bg-yellow-50';
      case 'completed': return 'border-gray-500 bg-gray-50';
      case 'alert': return 'border-red-500 bg-red-50';
      default: return 'border-gray-300 bg-white';
    }
  };

  const getStatusText = (status) => {
    switch (status) {
      case 'active': return { text: '输液中', color: 'text-green-800 bg-green-100' };
      case 'warning': return { text: '即将结束', color: 'text-yellow-800 bg-yellow-100' };
      case 'completed': return { text: '已完成', color: 'text-gray-800 bg-gray-100' };
      case 'alert': return { text: '异常报警', color: 'text-red-800 bg-red-100' };
      default: return { text: '未知', color: 'text-gray-800 bg-gray-100' };
    }
  };

  const PatientCard = ({ patient }) => {
    const percentage = (patient.remainingVolume / patient.totalVolume) * 100;
    const statusInfo = getStatusText(patient.status);

    return (
      <div className={`bg-white rounded-xl shadow-lg overflow-hidden border-l-8 ${getStatusColor(patient.status)} transition-all hover:shadow-2xl hover:-translate-y-1`}>
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
            </div>

            {/* 详细信息 */}
            <div className="flex-1 space-y-2">
              <p className="text-sm text-gray-500 font-medium">{patient.medication}</p>
              
              <div className="space-y-1">
                <p className="text-sm font-semibold text-gray-700">
                  当前滴速: <span className={patient.currentRate > 100 ? 'text-red-600 font-bold' : 'text-blue-600'}>
                    {patient.currentRate} 滴/分钟
                  </span>
                </p>
                
                <div className="flex items-center gap-1">
                  <Clock className="w-3 h-3 text-gray-500" />
                  <p className="text-sm font-semibold text-gray-700">
                    预计剩余: <span className={patient.estimatedTime < 10 ? 'text-yellow-600' : 'text-blue-600'}>
                      {patient.estimatedTime} 分钟
                    </span>
                  </p>
                </div>
              </div>

              {/* 进度条 */}
              <div className="w-full bg-gray-200 rounded-full h-2.5">
                <div 
                  className={`h-2.5 rounded-full transition-all duration-1000 ${
                    percentage > 50 ? 'bg-blue-500' : 
                    percentage > 20 ? 'bg-yellow-500' : 'bg-red-500'
                  }`}
                  style={{ width: `${percentage}%` }}
                ></div>
              </div>
              
              <p className="text-xs text-right text-gray-500">
                {patient.remainingVolume}ml / {patient.totalVolume}ml
              </p>
            </div>
          </div>
          
          {/* 底部状态和控制 */}
          <div className="mt-4 pt-4 border-t border-gray-200 flex justify-between items-center">
            <span className={`px-3 py-1 text-sm font-semibold rounded-full ${statusInfo.color} ${
              patient.status === 'warning' || patient.status === 'alert' ? 'animate-pulse' : ''
            }`}>
              状态: {statusInfo.text}
            </span>
            
            <div className="flex items-center gap-2">
              <button className={`text-sm font-semibold py-1 px-3 rounded-lg shadow-md transition-all ${
                patient.status === 'completed' 
                  ? 'bg-gray-400 text-white cursor-not-allowed' 
                  : 'bg-teal-500 text-white hover:bg-teal-600'
              }`}>
                自动夹断: {patient.status === 'completed' ? '已执行' : '开启'}
              </button>
            </div>
          </div>

          {/* 设备状态指示器 */}
          <div className="mt-3 flex justify-between text-xs text-gray-500">
            <div className="flex items-center gap-1">
              <Wifi className="w-3 h-3 text-green-500" />
              <span>WiFi 连接正常</span>
            </div>
            <div className="flex items-center gap-1">
              <Battery className="w-3 h-3 text-green-500" />
              <span>电量 85%</span>
            </div>
            <span>更新: {patient.lastUpdate.toLocaleTimeString()}</span>
          </div>
        </div>
      </div>
    );
  };

  const activeCount = patients.filter(p => p.status === 'active').length;
  const warningCount = patients.filter(p => p.status === 'warning').length;
  const alertCount = patients.filter(p => p.status === 'alert').length;

  return (
    <div className="min-h-screen bg-gray-100">
      <div className="container mx-auto p-4 md:p-8">
        {/* 头部 */}
        <header className="text-center mb-8">
          <h1 className="text-3xl font-bold text-gray-800 mb-2">护士站输液监控中心</h1>
          <p className="text-gray-500 mb-4">实时监控各病床输液状态</p>
          
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
