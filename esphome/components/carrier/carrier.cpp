#include "carrier.h"
#include "esphome/core/log.h"

namespace esphome {
namespace carrier {

static const char *TAG = "carrier.climate";

// Carrier (42NQV035G / 38NYV035H2) timing constants (remote control P/N WH-L05SE)
const uint32_t CARRIER_AIRCON1_HDR_MARK = 4320;
const uint32_t CARRIER_AIRCON1_HDR_SPACE = 4350;
const uint32_t CARRIER_AIRCON1_BIT_MARK = 500;
const uint32_t CARRIER_AIRCON1_ONE_SPACE = 1650;
const uint32_t CARRIER_AIRCON1_ZERO_SPACE = 550;
const uint32_t CARRIER_AIRCON1_MSG_SPACE = 7400;

// Carrier codes
const uint8_t CARRIER_AIRCON1_MODE_AUTO = 0x00; // Operating mode
const uint8_t CARRIER_AIRCON1_MODE_HEAT = 0xC0;
const uint8_t CARRIER_AIRCON1_MODE_COOL = 0x80;
const uint8_t CARRIER_AIRCON1_MODE_DRY = 0x40;
const uint8_t CARRIER_AIRCON1_MODE_FAN = 0x20;
const uint8_t CARRIER_AIRCON1_MODE_OFF = 0xE0; // Power OFF
const uint8_t CARRIER_AIRCON1_FAN_AUTO = 0x00; // Fan speed
const uint8_t CARRIER_AIRCON1_FAN1 = 0x02;
const uint8_t CARRIER_AIRCON1_FAN2 = 0x06;
const uint8_t CARRIER_AIRCON1_FAN3 = 0x01;
const uint8_t CARRIER_AIRCON1_FAN4 = 0x05;
const uint8_t CARRIER_AIRCON1_FAN5 = 0x03;


// Carrier (42MCA009515LS) timing constants (remote control P/N R11CG/E)
const uint32_t CARRIER_AIRCON2_HDR_MARK = 4510;
const uint32_t CARRIER_AIRCON2_HDR_SPACE = 4470;
const uint32_t CARRIER_AIRCON2_BIT_MARK = 600;
const uint32_t CARRIER_AIRCON2_ONE_SPACE = 1560;
const uint32_t CARRIER_AIRCON2_ZERO_SPACE = 500;

const uint8_t CARRIER_AIRCON2_MODE_AUTO = 0x10; // Operating mode
const uint8_t CARRIER_AIRCON2_MODE_COOL = 0x00;
const uint8_t CARRIER_AIRCON2_MODE_DRY = 0x20;
const uint8_t CARRIER_AIRCON2_MODE_FAN = 0x20;
const uint8_t CARRIER_AIRCON2_MODE_HEAT = 0x30;
const uint8_t CARRIER_AIRCON2_MODE_OFF = 0x00; // Power OFF
const uint8_t CARRIER_AIRCON2_MODE_ON = 0x20; // Power ON
const uint8_t CARRIER_AIRCON2_FAN_DRY_AUTO = 0x00; // Fan speed, AUTO or DRY modes
const uint8_t CARRIER_AIRCON2_FAN1 = 0x01;
const uint8_t CARRIER_AIRCON2_FAN2 = 0x02;
const uint8_t CARRIER_AIRCON2_FAN3 = 0x04;
const uint8_t CARRIER_AIRCON2_FAN_AUTO = 0x05;
const uint8_t CARRIER_AIRCON2_FAN_OFF = 0x06;


const uint32_t MODEL_CARRIER_MCA = 1;
const uint32_t MODEL_QLIMA_1 = 2;
const uint32_t MODEL_QLIMA_2 = 3;

const uint8_t CARRIER_TEMP_MIN = 17;
const uint8_t CARRIER_TEMP_MAX = 30;

climate::ClimateTraits CarrierClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_current_temperature(this->sensor_ != nullptr);
  traits.set_supports_auto_mode(true);
  traits.set_supports_cool_mode(this->supports_cool_);
  traits.set_supports_heat_mode(this->supports_heat_);
  traits.set_supports_two_point_target_temperature(false);
  traits.set_supports_away(false);
  traits.set_visual_min_temperature(CARRIER_TEMP_MIN);
  traits.set_visual_max_temperature(CARRIER_TEMP_MAX);
  traits.set_visual_temperature_step(1);
  return traits;
}

void CarrierClimate::setup() {
  if (this->sensor_) {
    this->sensor_->add_on_state_callback([this](float state) {
      this->current_temperature = state;
      // current temperature changed, publish state
      this->publish_state();
    });
    this->current_temperature = this->sensor_->state;
  } else
    this->current_temperature = NAN;
  // restore set points
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    // restore from defaults
    this->mode = climate::CLIMATE_MODE_OFF;
    // initialize target temperature to some value so that it's not NAN
    //this->target_temperature = roundf(this->current_temperature);
    this->target_temperature = 26;
  }
}

void CarrierClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value())
    this->mode = *call.get_mode();
  if (call.get_target_temperature().has_value())
    this->target_temperature = *call.get_target_temperature();

  this->transmit_state_();
  this->publish_state();
}

void CarrierClimate::transmit_state_() {
  uint8_t power_mode = CARRIER_AIRCON2_MODE_ON;
  uint8_t operating_mode = CARRIER_AIRCON2_MODE_COOL;
  uint8_t fan_speed = CARRIER_AIRCON2_FAN_AUTO;
  uint8_t temperature;
  static const uint8_t temperatures[] = {0, 8, 12, 4, 6, 14, 10, 2, 3, 11, 9, 1, 5, 13, 7};


  switch (this->mode) {
    case climate::CLIMATE_MODE_HEAT:
      operating_mode = CARRIER_AIRCON2_MODE_HEAT;
      break;
    case climate::CLIMATE_MODE_COOL:
      operating_mode = CARRIER_AIRCON2_MODE_COOL;
      break;
    case climate::CLIMATE_MODE_AUTO:
      operating_mode = CARRIER_AIRCON2_MODE_AUTO;
      break;
    case climate::CLIMATE_MODE_OFF:
      operating_mode = CARRIER_AIRCON2_MODE_OFF;
      break;
  }

  temperature = (uint8_t) roundf(clamp(this->target_temperature, CARRIER_TEMP_MIN, CARRIER_TEMP_MAX));

  uint8_t remote_state[] = {0x4D, 0xB2, (uint8_t)(power_mode | fan_speed), 0x00,
                           (uint8_t)(operating_mode | (temperatures[(temperature - 17)])), 0x00};
  remote_state[3] = ~remote_state[2];
  remote_state[5] = ~remote_state[4];

  ESP_LOGV(TAG, "Sending carrier code: %u", remote_state);

  auto transmit = this->transmitter_->transmit();
  auto data = transmit.get_data();

  data->set_carrier_frequency(38000);

  // Header
  data->mark(CARRIER_AIRCON2_HDR_MARK);
  data->space(CARRIER_AIRCON2_HDR_SPACE);
  // Data
  // Payload
  for (uint8_t i : remote_state)
    for (uint8_t j = 0; j < 8; j++) {
      data->mark(CARRIER_AIRCON2_BIT_MARK);
      bool bit = i & (1 << j);
      data->space(bit ? CARRIER_AIRCON2_ONE_SPACE : CARRIER_AIRCON2_ZERO_SPACE);
    }
  transmit.perform();
  // New Header
  data->reset();
  data->set_carrier_frequency(38000);
  data->mark(CARRIER_AIRCON2_BIT_MARK);
  data->space(CARRIER_AIRCON2_HDR_SPACE);
  data->mark(CARRIER_AIRCON2_HDR_MARK);
  data->space(CARRIER_AIRCON2_HDR_SPACE);

  // Data
  for (uint8_t i : remote_state)
    for (uint8_t j = 0; j < 8; j++) {
      data->mark(CARRIER_AIRCON2_BIT_MARK);
      bool bit = i & (1 << j);
      data->space(bit ? CARRIER_AIRCON2_ONE_SPACE : CARRIER_AIRCON2_ZERO_SPACE);
    }

  // End mark
  data->mark(CARRIER_AIRCON2_BIT_MARK);
  data->space(0);

  transmit.perform();
}

}  // namespace carrier
}  // namespace esphome
