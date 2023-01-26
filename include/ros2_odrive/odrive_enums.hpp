/*
* Copyright (C) 2021 Alexander Junk <dev@junk.technology>
* 
* This code is based on the ros_odrive repository by Ioannis Kokkoris
* (https://github.com/johnkok/ros_odrive), which is distributed under the
* MIT license.
* 
* This program is free software: you can redistribute it and/or modify it 
* under the terms of the GNU Lesser General Public License as published 
* by the Free Software Foundation, either version 3 of the License, or 
* (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. 
* See the GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License 
* along with this program. If not, see <https://www.gnu.org/licenses/>. 
*
*/

#ifndef ODRIVE_ENUMS_HPP_
#define ODRIVE_ENUMS_HPP_

// ODrive.Can.Protocol
#define PROTOCOL_SIMPLE                           0

namespace utils
{

    namespace details
    {

        template <typename E>
        using enable_enum_t = typename std::enable_if<std::is_enum<E>::value,
                                                      typename std::underlying_type<E>::type>::type;

    } // namespace details

    template <typename E>
    constexpr inline details::enable_enum_t<E> underlying_value(E e) noexcept
    {
        return static_cast<typename std::underlying_type<E>::type>(e);
    }

    template <typename E, typename T>
    constexpr inline typename std::enable_if<std::is_enum<E>::value &&
                                                 std::is_integral<T>::value,
                                             E>::type
    to_enum(T value) noexcept
    {
        return static_cast<E>(value);
    }

} // namespace utils


struct AXIS_FUNC {
    static const std::string CLEAR_ERRORS;
};


// ODrive.Axis.AxisState
enum class AXIS_STATE : uint32_t {
    UNDEFINED                     = 0,
    IDLE                          = 1,
    STARTUP_SEQUENCE              = 2,
    FULL_CALIBRATION_SEQUENCE     = 3,
    MOTOR_CALIBRATION             = 4,
    SENSORLESS_CONTROL            = 5,
    ENCODER_INDEX_SEARCH          = 6,
    ENCODER_OFFSET_CALIBRATION    = 7,
    CLOSED_LOOP_CONTROL           = 8,
    LOCKIN_SPIN                   = 9,
    ENCODER_DIR_FIND              = 10,
    HOMING                        = 11
};

// ODrive.ThermistorCurrentLimiter.Error
enum class THERMISTOR_CURRENT_LIMITER_ERROR : int32_t {
    NONE       = 0x00000000,
    OVER_TEMP  = 0x00000001
};

// ODrive.Encoder.Mode
enum class ENCODER_MODE : uint32_t {
    INCREMENTAL  = 0,
    HALL         = 1,
    SINCOS       = 2,
    SPI_ABS_CUI  = 256,
    SPI_ABS_AMS  = 257,
    SPI_ABS_AEAT = 258
};

// ODrive.Controller.ControlMode
enum class CONTROL_MODE : int32_t {
    VOLTAGE_CONTROL             = 0,
    TORQUE_CONTROL              = 1,
    VELOCITY_CONTROL            = 2,
    POSITION_CONTROL            = 3
};

// ODrive.Controller.InputMode
enum class INPUT_MODE : int32_t {
    INACTIVE                     =  0,
    PASSTHROUGH                  =  1,
    VEL_RAMP                     =  2,
    POS_FILTER                   =  3,
    MIX_CHANNELS                 =  4,
    TRAP_TRAJ                    =  5,
    TORQUE_RAMP                  =  6,
    MIRROR                       =  7
};

// ODrive.Motor.MotorType
enum class MOTOR_TYPE : int32_t {
    HIGH_CURRENT                  = 0,
    GIMBAL                        = 2,
    ACIM                          = 3
};

// ODrive.Can.Error
enum class CAN_ERROR : int32_t {
    NONE                           = 0x00000000,
    DUPLICATE_CAN_IDS              = 0x00000001
};

// ODrive.Axis.Error
enum class AXIS_ERROR : int32_t {
    NONE                          = 0x00000000,
    INVALID_STATE                 = 0x00000001,
    DC_BUS_UNDER_VOLTAGE          = 0x00000002,
    DC_BUS_OVER_VOLTAGE           = 0x00000004,
    CURRENT_MEASUREMENT_TIMEOUT   = 0x00000008,
    BRAKE_RESISTOR_DISARMED       = 0x00000010,
    MOTOR_DISARMED                = 0x00000020,
    MOTOR_FAILED                  = 0x00000040,
    SENSORLESS_ESTIMATOR_FAILED   = 0x00000080,
    ENCODER_FAILED                = 0x00000100,
    CONTROLLER_FAILED             = 0x00000200,
    POS_CTRL_DURING_SENSORLESS    = 0x00000400,
    WATCHDOG_TIMER_EXPIRED        = 0x00000800,
    MIN_ENDSTOP_PRESSED           = 0x00001000,
    MAX_ENDSTOP_PRESSED           = 0x00002000,
    ESTOP_REQUESTED               = 0x00004000,
    HOMING_WITHOUT_ENDSTOP        = 0x00020000,
    OVER_TEMP                     = 0x00040000
};

// ODrive.Axis.LockinState
enum class LOCKIN_STATE : int32_t {
    INACTIVE              = 0,
    RAMP                  = 1,
    ACCELERATE            = 2,
    VEL                   = 3
};

// ODrive.Motor.Error
enum class MOTOR_ERROR : int32_t {

    NONE                          = 0x00000000,
    PHASE_RESISTANCE_OUT_OF_RANGE = 0x00000001,
    PHASE_INDUCTANCE_OUT_OF_RANGE = 0x00000002,
    ADC_FAILED                    = 0x00000004,
    DRV_FAULT                     = 0x00000008,
    CONTROL_DEADLINE_MISSED       = 0x00000010,
    NOT_IMPLEMENTED_MOTOR_TYPE    = 0x00000020,
    BRAKE_CURRENT_OUT_OF_RANGE    = 0x00000040,
    MODULATION_MAGNITUDE          = 0x00000080,
    BRAKE_DEADTIME_VIOLATION      = 0x00000100,
    UNEXPECTED_TIMER_CALLBACK     = 0x00000200,
    CURRENT_SENSE_SATURATION      = 0x00000400,
    CURRENT_LIMIT_VIOLATION       = 0x00001000,
    BRAKE_DUTY_CYCLE_NAN          = 0x00002000,
    DC_BUS_OVER_REGEN_CURRENT     = 0x00004000,
    DC_BUS_OVER_CURRENT           = 0x00008000
};

// ODrive.Motor.ArmedState
enum class ARMED_STATE : int32_t{
    DISARMED                     = 0,
    WAITING_FOR_TIMINGS          = 1,
    WAITING_FOR_UPDATE           = 2,
    ARMED                        = 3
};

// ODrive.Motor.GateDriver.DrvFault
enum class DRV_FAULT : int32_t{
    NO_FAULT                        = 0x00000000,
    FET_LOW_C_OVERCURRENT           = 0x00000001,
    FET_HIGH_C_OVERCURRENT          = 0x00000002,
    FET_LOW_B_OVERCURRENT           = 0x00000004,
    FET_HIGH_B_OVERCURRENT          = 0x00000008,
    FET_LOW_A_OVERCURRENT           = 0x00000010,
    FET_HIGH_A_OVERCURRENT          = 0x00000020,
    OVERTEMPERATURE_WARNING         = 0x00000040,
    OVERTEMPERATURE_SHUTDOWN        = 0x00000080,
    P_VDD_UNDERVOLTAGE              = 0x00000100,
    G_VDD_UNDERVOLTAGE              = 0x00000200,
    G_VDD_OVERVOLTAGE               = 0x00000400
};

// ODrive.Controller.Error
enum class CONTROLLER_ERROR : int32_t{
    NONE                     = 0x00000000,
    OVERSPEED                = 0x00000001,
    INVALID_INPUT_MODE       = 0x00000002,
    UNSTABLE_GAIN            = 0x00000004,
    INVALID_MIRROR_AXIS      = 0x00000008,
    INVALID_LOAD_ENCODER     = 0x00000010,
    INVALID_ESTIMATE         = 0x00000020
};

// ODrive.Encoder.Error
enum class ENCODER_ERROR : int32_t{
    NONE                        = 0x00000000,
    UNSTABLE_GAIN               = 0x00000001,
    CPR_POLEPAIRS_MISMATCH      = 0x00000002,
    NO_RESPONSE                 = 0x00000004,
    UNSUPPORTED_ENCODER_MODE    = 0x00000008,
    ILLEGAL_HALL_STATE          = 0x00000010,
    INDEX_NOT_FOUND_YET         = 0x00000020,
    ABS_SPI_TIMEOUT             = 0x00000040,
    ABS_SPI_COM_FAIL            = 0x00000080,
    ABS_SPI_NOT_READY           = 0x00000100
};

// ODrive.SensorlessEstimator.Error
enum class SENSORLESS_ESTIMATOR_ERROR : int32_t{
    NONE           = 0x00000000,
    UNSTABLE_GAIN  = 0x00000001
};


#endif

