// Firmware for an OpenTherm/plus interface
//
#include <Arduino.h>
#include "opentherm.h"

#define OT_BUS_IN 13 //D7
#define OT_BUS_OUT 5 //D1
#define LED 2 //D4

#define MAX_ALLOWED_OT_TIMEOUTS 10

bool requested_ch_enabled = false;
float requested_ch_set_point = 0.0;
float requested_max_modulation = 100.0;

char command_buffer[20];
unsigned int received_bytes = 0;
unsigned int debug_level = 0;
unsigned int ot_communication_error_count = 0;

struct {
    bool fault = false;
    bool ch_on = false;
    bool dhw_on = false;
    bool flame_on = false;
    float modulation_level = 0;
    float boiler_flow_water_temp = 0;
    float max_ch_set_point = 0;
    float ch_water_pressure = 0;
} boiler_state;

unsigned int message_loop_index = 0;

#define MESSAGE_LOOP_LENGTH 7
OpenthermData message_loop_list[MESSAGE_LOOP_LENGTH] = {
        {OT_MSGTYPE_READ_DATA, OT_MSGID_STATUS, 0, 0},
        {OT_MSGTYPE_READ_DATA, OT_MSGID_SLAVE_CONFIG, 0, 0},
        {OT_MSGTYPE_READ_DATA, OT_MSGID_MODULATION_LEVEL, 0, 0},
        {OT_MSGTYPE_READ_DATA, OT_MSGID_CH_WATER_PRESSURE, 0, 0},
        {OT_MSGTYPE_READ_DATA, OT_MSGID_BOILER_FLOW_WATER_TEMP, 0, 0},
        {OT_MSGTYPE_READ_DATA, OT_MSGID_MAX_CH_SETPOINT, 0, 0},
        {OT_MSGTYPE_WRITE_DATA, OT_MSGID_CH_SETPOINT, 0, 0},
};

void setup()
{
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    pinMode(OT_BUS_IN, INPUT);
    digitalWrite(OT_BUS_IN, HIGH);
    digitalWrite(OT_BUS_OUT, HIGH);
    pinMode(OT_BUS_OUT, OUTPUT);
    Serial.begin(115200);
    delay(3000);
    Serial.print("Hello, I am an OpenTherm/plus interface");
}

unsigned int build_master_status_message()
{
    bool enable_domestic_hot_water = true;
    bool enable_cooling = true;
    bool enable_outside_temperature_compensation = false;
    bool enable_central_heating_channel_2 = false;

    unsigned int data = requested_ch_enabled |
            (enable_domestic_hot_water << 1) |
            (enable_cooling << 2) |
            (enable_outside_temperature_compensation << 3) |
            (enable_central_heating_channel_2 << 4);

    return data << 8;
}

OpenthermData get_next_ot_message_to_send()
{
    OpenthermData next_message = message_loop_list[message_loop_index];

    if (next_message.id == OT_MSGID_STATUS) {
        next_message.u16(build_master_status_message());
    } else if (next_message.id == OT_MSGID_CH_SETPOINT) {
        next_message.f88(requested_ch_set_point);
    } else if (next_message.id == OT_MSGID_MAX_RELATIVE_MODULATION) {
        next_message.f88(requested_max_modulation);
    }

    return next_message;
}

void print_status()
{
    Serial.print("=== fault:");
    Serial.println(boiler_state.fault);
    Serial.print("=== ch_on:");
    Serial.println(boiler_state.ch_on);
    Serial.print("=== dhw_on:");
    Serial.println(boiler_state.dhw_on);
    Serial.print("=== flame_on:");
    Serial.println(boiler_state.flame_on);
    Serial.print("=== modulation_level:");
    Serial.println(boiler_state.modulation_level);
    Serial.print("=== boiler_flow_water_temp:");
    Serial.println(boiler_state.boiler_flow_water_temp);
    Serial.print("=== max_ch_set_point:");
    Serial.println(boiler_state.max_ch_set_point);
    Serial.print("=== ch_water_pressure:");
    Serial.println(boiler_state.ch_water_pressure);
    Serial.println();
}

void handle_ot_response(OpenthermData response)
{
    message_loop_index = (message_loop_index + 1) % MESSAGE_LOOP_LENGTH;

    switch (response.id) {
        case OT_MSGID_STATUS:
            boiler_state.fault = bitRead(response.valueLB, 0);
            boiler_state.ch_on = bitRead(response.valueLB, 1);
            boiler_state.dhw_on = bitRead(response.valueLB, 2);
            boiler_state.flame_on = bitRead(response.valueLB, 3);
            break;
        case OT_MSGID_SLAVE_CONFIG:
            break;
        case OT_MSGID_MODULATION_LEVEL:
            boiler_state.modulation_level = response.f88();
            break;
        case OT_MSGID_BOILER_FLOW_WATER_TEMP:
            boiler_state.boiler_flow_water_temp = response.f88();
            break;
        case OT_MSGID_CH_WATER_PRESSURE:
            boiler_state.ch_water_pressure = response.f88();
            break;
        case OT_MSGID_MAX_CH_SETPOINT:
            boiler_state.max_ch_set_point = response.f88();
            break;
    }

    if (debug_level > 0) {
        Serial.print("<<< ");
        OPENTHERM::printToSerial(response);
        Serial.println();
    }

    if (message_loop_index == 0) {
        print_status();
    }

    delay(110);
}

void handle_ot_conversation()
{
    OpenthermData received_ot_message{};

    if (OPENTHERM::isIdle()) {
        digitalWrite(LED, LOW);
        OpenthermData message_to_send = get_next_ot_message_to_send();
        if (debug_level > 0) {
            Serial.print(">>> ");
            OPENTHERM::printToSerial(message_to_send);
            Serial.println();
        }
        OPENTHERM::send(OT_BUS_OUT, message_to_send);
    } else if (OPENTHERM::isSent()) {
        OPENTHERM::listen(OT_BUS_IN, 300);
    } else if (OPENTHERM::getMessage(received_ot_message)) {
        OPENTHERM::stop();
        digitalWrite(LED, HIGH);
        ot_communication_error_count = 0;
        handle_ot_response(received_ot_message);
    } else if (OPENTHERM::isError()) {
        OPENTHERM::stop();
        ot_communication_error_count++;
        if (ot_communication_error_count > MAX_ALLOWED_OT_TIMEOUTS) {
            digitalWrite(LED, HIGH);
            ot_communication_error_count = MAX_ALLOWED_OT_TIMEOUTS;
            Serial.println("E: Too many consecutive OT response timeouts");
            delay(5000);
        }
        if (debug_level > 0) Serial.println("W: Timeout waiting for OT slave response");
        digitalWrite(LED, HIGH);
    }
}

void parse_received_command()
{
    char command[8];
    float param_1 = 0.0;
    float param_2 = 100.0;

    int parsed_count = sscanf(command_buffer, "%s %f %f", command, &param_1, &param_2);

    if (parsed_count < 1) goto exit_parse_error;

    if (strncmp(command, "on", 2) == 0) {
        if (parsed_count != 3 || param_1 > 100 || param_1 < 0 || param_2 < 0 || param_2 > 100) {
            goto exit_parse_error;
        }
        requested_ch_enabled = true;
        requested_ch_set_point = param_1;
        requested_max_modulation = param_2;
        Serial.print("==> Heating ON. Set point:");
        Serial.print(param_1);
        Serial.print(" Max. modulation:");
        Serial.print(param_2);
        Serial.println();
        return;
    } else if (strncmp(command_buffer, "off", 3) == 0) {
        if (parsed_count != 1) goto exit_parse_error;
        requested_ch_enabled = false;
        Serial.println("==> Heating OFF");
        return;
    } else if (strncmp(command_buffer, "debug", 5) == 0) {
        if (parsed_count != 2) goto exit_parse_error;
        debug_level = (param_1 > 0);
        Serial.print("==> Debug level");
        Serial.println(debug_level);
        return;
    }

exit_parse_error:
    Serial.println("E: Invalid command or arguments");
}

void check_for_serial_data()
{
    if (Serial.available() <= 0) {
        return;
    }

    char incoming_byte = Serial.read();

    if (received_bytes >= sizeof(command_buffer)) {
        Serial.println("E: Command overflow");
        received_bytes = 0;
    }

    if (incoming_byte == '\r') {
        command_buffer[received_bytes++] = '\0';
        parse_received_command();
        received_bytes = 0;
    } else {
        command_buffer[received_bytes++] = incoming_byte;
    }
}

void loop()
{
    check_for_serial_data();
    handle_ot_conversation();
}
