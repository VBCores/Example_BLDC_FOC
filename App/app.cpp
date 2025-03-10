#include "app.h"

#include <memory>

#include "tim.h"
#include "i2c.h"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "cordic.h"

#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/providers/G4CAN.h>

#include <uavcan/node/Mode_1_0.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/primitive/scalar/Real32_1_0.h>
#include <uavcan/primitive/scalar/Natural32_1_0.h>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/ASxxxx/AS5047P.hpp>

EEPROM eeprom(&hi2c4);

void setup_cordic() {
    CORDIC_ConfigTypeDef cordic_config {
        .Function = CORDIC_FUNCTION_COSINE,
        .Scale = CORDIC_SCALE_0,
        .InSize = CORDIC_INSIZE_32BITS,
        .OutSize = CORDIC_OUTSIZE_32BITS,
        .NbWrite = CORDIC_NBWRITE_1,
        .NbRead = CORDIC_NBREAD_2,
        .Precision = CORDIC_PRECISION_6CYCLES
    };
    HAL_IMPORTANT(HAL_CORDIC_Configure(&hcordic, &cordic_config));
}

AS5047P encoder(
    GpioPin(
        SPI1_NSS_GPIO_Port,
        SPI1_NSS_Pin
    ),
    &hspi1,
    false,
    0
);

static encoder_data enc_val = 0;
static int enc_rev = 0;
[[noreturn]] void app() {
    setup_cordic();
    start_timers();
    start_cyphal();

    while (!eeprom.is_connected()) {
        eeprom.delay();
    }
    eeprom.delay();

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    while(true) {
        cyphal_loop();
        encoder.update_value();
        enc_val = encoder.get_value();
        enc_rev = encoder.get_revolutions();
    }
}

TYPE_ALIAS(Natural32, uavcan_primitive_scalar_Natural32_1_0)
TYPE_ALIAS(Real32, uavcan_primitive_scalar_Real32_1_0)
static constexpr CanardPortID ENCODER_PORT = 7100;
static constexpr CanardPortID VOLTAGE_PORT = 5800;

/*
void in_loop_reporting(millis current_t) {
    static millis report_time = 0;
    EACH_N(current_t, report_time, 50, {
        Natural32::Type enc_msg = {};
        enc_msg.value = hall_sensor.get_value();
        static CanardTransferID enc_transfer_id = 0;
        get_interface()->send_msg<Natural32>(&enc_msg, ENCODER_PORT, &enc_transfer_id);
    })
}

class VoltageSub: public AbstractSubscription<Real32> {
public:
VoltageSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Real32>(interface, port_id) {};
    void handler(const Real32::Type& msg, CanardRxTransfer* _) override {
        motor->set_voltage_point(msg.value);
    }
};
*/

ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<1>> registers_handler;
//ReservedObject<VoltageSub> voltage_sub;

void setup_subscriptions() {
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    auto cyphal_interface = get_interface();
    const auto node_id = get_node_id();

    //voltage_sub.create(cyphal_interface, VOLTAGE_PORT + node_id);
    node_info_reader.create(
        cyphal_interface,
        "org.voltbro.bldc_foc",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        0
    );
    registers_handler.create(
        std::array<RegisterDefinition, 1>{{
            {
                "motor.is_on",
                [](
                    const uavcan_register_Value_1_0& v_in,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse::Type& response
                ){
                    static bool value = false;
                    if (v_in._tag_ == 3) {
                        value = v_in.bit.value.bitpacked[0] == 1;
                    }
                    else {
                        // TODO: report error
                    }

                    //motor->set_state(value);

                    response.persistent = true;
                    response._mutable = true;
                    v_out._tag_ = 3;
                    //v_out.bit.value.bitpacked[0] = motor->is_on();
                    v_out.bit.value.count = 1;
                }
            }
        }},
        cyphal_interface
    );

    static FDCAN_FilterTypeDef sFilterConfig;
    uint32_t filter_index = 0;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        node_info_reader->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        registers_handler->make_filter(node_id)
    ))

    /*
    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        voltage_sub->make_filter(node_id)
    ))
    */
}
