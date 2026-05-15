#pragma once

#include <stdint.h>

#include <SPI.h>
#include <SimpleFOC.h>

#include "common/sensors/mt6701_ssi_sensor.h"

// SimpleFOC Sensor 适配类：内部使用 common 的 MT6701 快速读取器。
class MasterMt6701Sensor : public Sensor {
public:
    explicit MasterMt6701Sensor(int cs_pin);

    void init(SPIClass *spi = &SPI);
    uint32_t lastFrame() const;
    uint16_t rawAngle() const;
    uint8_t magneticStatus() const;
    uint32_t lastReadDurationUs() const;

protected:
    float getSensorAngle() override;

private:
    Mt6701SsiFastReader reader_;
};

bool setupMasterEncoderHardware();
void setMasterKnobMotorReadyForEncoder(bool ready);
MasterMt6701Sensor &masterKnobSensor();
void getMasterEncoderDiagnostics(uint32_t &frame, uint16_t &raw_angle, uint8_t &magnetic_status);
float readMasterKnobAngleDeg();
