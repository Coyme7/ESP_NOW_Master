#include <shared_types.h>

uint8_t slaveAddress[] = {0x24, 0x58, 0x7c, 0xd0, 0xab, 0x2c};
ControlPacket txPacket, rxPacket;

// --- Core 0: ESP-NOW 回调 ---
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&rxPacket, incomingData, sizeof(rxPacket));
    // 将收到的从机状态更新到共享内存
    sysData.slave_x_pos = rxPacket.x;
    sysData.slave_y_pos = rxPacket.y;
    sysData.is_boundary_hit = rxPacket.button; // 暂时用 button 位模拟撞墙信号
}

// ==========================================
// 核心 1 (Core 1): FOC 与 力觉反馈算法核
// ==========================================
void IRAM_ATTR task_foc_loop(void *pvParameters) {
    // 模拟 FOC 初始化...
    float simulated_knob_x = 0; // 模拟我们手捏着的旋钮物理位置
    
    while(true) {
        // 1. 获取当前模式 (无锁读取)
        uint8_t mode = sysData.current_mode;
        
        // 2. 执行力觉算法
        if (mode == MODE_COLLAB_DRAW) {
            // 协同模式：模拟人手转动旋钮
            simulated_knob_x += 0.01f; 
            
            // 重点：死区力觉反馈！
            if (sysData.is_boundary_hit) {
                // 如果从端撞墙，这里应该给 FOC 电流环下发反向大电流产生“硬墙”手感
                // motor.target_voltage = Kp * (boundary_pos - simulated_knob_x);
            } else {
                // 丝滑悬空手感
                // motor.target_voltage = 0;
            }
            sysData.master_x_pos = simulated_knob_x; // 更新共享内存
            
        } else if (mode == MODE_AUTO_DRAW) {
            // 逆向反馈模式：旋钮被从端强行牵引
            // 虚拟弹簧力：旋钮自动追随从端的位置
            float error = sysData.slave_x_pos - simulated_knob_x;
            // motor.target_voltage = error * SPRING_KP; 
            simulated_knob_x += error * 0.1f; // 模拟旋钮被拉动
            sysData.master_x_pos = simulated_knob_x;
            
        } else if (mode == MODE_BLE_MEDIA) {
            // 蓝牙模式：模拟齿轮手感 (每隔一定角度产生阻力)
            // if (fmod(simulated_knob_x, 30.0f) < 1.0f) motor.target = DETENT_FORCE;
            simulated_knob_x += 0.05f; // 模拟用户在把玩
            sysData.master_x_pos = simulated_knob_x;
        }

        // 真实情况这里是 esp_timer 硬件中断等待，这里为了模拟暂用 delay
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// ==========================================
// 核心 0 (Core 0): 通信与 UI 调度核
// ==========================================
void task_comm_loop(void *pvParameters) {
    uint32_t packet_id = 0;
    while(true) {
        // 1. 从共享内存打包数据
        txPacket.x = sysData.master_x_pos;
        txPacket.y = sysData.master_y_pos;
        txPacket.mode = sysData.current_mode;
        txPacket.packet_id = packet_id++;
        
        // 2. 发送给从端
        esp_now_send(slaveAddress, (uint8_t *) &txPacket, sizeof(txPacket));
        
        // 3. 串口监控 (极其重要的数据流验证)
        Serial.printf("[Master] Mode: %d | Knob: %.2f | Slave Feedback: %.2f | Hit: %d\n", 
                      sysData.current_mode, sysData.master_x_pos, sysData.slave_x_pos, sysData.is_boundary_hit);
                      
        // 模拟模式切换：每 5 秒切换一种工作模式
        if (millis() % 15000 < 5000) sysData.current_mode = MODE_COLLAB_DRAW;
        else if (millis() % 15000 < 10000) sysData.current_mode = MODE_AUTO_DRAW;
        else sysData.current_mode = MODE_BLE_MEDIA;

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz 通讯率
    }
}

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_recv_cb(onDataRecv);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, slaveAddress, 6);
    esp_now_add_peer(&peerInfo);

    // 【架构灵魂】将两个任务死死钉在对应的核心上！
    // 参数：任务名, 栈大小, 传参, 优先级, 句柄, 核心编号
    xTaskCreatePinnedToCore(task_comm_loop, "CommTask", 4096, NULL, 1, NULL, 0); // Core 0
    xTaskCreatePinnedToCore(task_foc_loop,  "FOCTask",  8192, NULL, configMAX_PRIORITIES - 1, NULL, 1); // Core 1
}
