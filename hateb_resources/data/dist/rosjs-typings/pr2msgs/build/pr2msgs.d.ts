declare namespace PR2Msgs {
    class AccelerometerState extends ROSLIB.Message {
        header: StdMsgs.Header;
        samples: GeometryMsgs.Vector3[];
        constructor(values: {
            header: StdMsgs.Header;
            samples: GeometryMsgs.Vector3[];
        });
    }
    class AccessPoint extends ROSLIB.Message {
        header: StdMsgs.Header;
        essid: string;
        macaddr: string;
        signal: number;
        noise: number;
        snr: number;
        channel: number;
        rate: string;
        tx_power: string;
        quality: number;
        constructor(values: {
            header: StdMsgs.Header;
            essid: string;
            macaddr: string;
            signal: number;
            noise: number;
            snr: number;
            channel: number;
            rate: string;
            tx_power: string;
            quality: number;
        });
    }
    class BatteryServer2 extends ROSLIB.Message {
        header: StdMsgs.Header;
        MAX_BAT_COUNT: number;
        MAX_BAT_REG: number;
        id: number;
        last_system_update: {
            secs: number;
            nsecs: number;
        };
        time_left: {
            secs: number;
            nsecs: number;
        };
        average_charge: number;
        message: string;
        last_controller_update: {
            secs: number;
            nsecs: number;
        };
        battery: PR2Msgs.BatteryState2[];
        constructor(values: {
            header: StdMsgs.Header;
            id: number;
            last_system_update: {
                secs: number;
                nsecs: number;
            };
            time_left: {
                secs: number;
                nsecs: number;
            };
            average_charge: number;
            message: string;
            last_controller_update: {
                secs: number;
                nsecs: number;
            };
            battery: PR2Msgs.BatteryState2[];
        });
    }
    class BatteryState2 extends ROSLIB.Message {
        present: boolean;
        charging: boolean;
        discharging: boolean;
        power_present: boolean;
        power_no_good: boolean;
        inhibited: boolean;
        last_battery_update: {
            secs: number;
            nsecs: number;
        };
        battery_register: number[];
        battery_update_flag: boolean[];
        battery_register_update: {
            secs: number;
            nsecs: number;
        }[];
        constructor(values: {
            present: boolean;
            charging: boolean;
            discharging: boolean;
            power_present: boolean;
            power_no_good: boolean;
            inhibited: boolean;
            last_battery_update: {
                secs: number;
                nsecs: number;
            };
            battery_register: number[];
            battery_update_flag: boolean[];
            battery_register_update: {
                secs: number;
                nsecs: number;
            }[];
        });
    }
    class DashboardState extends ROSLIB.Message {
        motors_halted: StdMsgs.Bool;
        motors_halted_valid: boolean;
        power_board_state: PR2Msgs.PowerBoardState;
        power_board_state_valid: boolean;
        power_state: PR2Msgs.PowerState;
        power_state_valid: boolean;
        access_point: PR2Msgs.AccessPoint;
        access_point_valid: boolean;
        constructor(values: {
            motors_halted: StdMsgs.Bool;
            motors_halted_valid: boolean;
            power_board_state: PR2Msgs.PowerBoardState;
            power_board_state_valid: boolean;
            power_state: PR2Msgs.PowerState;
            power_state_valid: boolean;
            access_point: PR2Msgs.AccessPoint;
            access_point_valid: boolean;
        });
    }
    class GPUStatus extends ROSLIB.Message {
        header: StdMsgs.Header;
        product_name: string;
        pci_device_id: string;
        pci_location: string;
        display: string;
        driver_version: string;
        temperature: number;
        fan_speed: number;
        gpu_usage: number;
        memory_usage: number;
        constructor(values: {
            header: StdMsgs.Header;
            product_name: string;
            pci_device_id: string;
            pci_location: string;
            display: string;
            driver_version: string;
            temperature: number;
            fan_speed: number;
            gpu_usage: number;
            memory_usage: number;
        });
    }
    class LaserScannerSignal extends ROSLIB.Message {
        header: StdMsgs.Header;
        signal: number;
        constructor(values: {
            header: StdMsgs.Header;
            signal: number;
        });
    }
    class LaserTrajCmd extends ROSLIB.Message {
        header: StdMsgs.Header;
        profile: string;
        position: number[];
        time_from_start: {
            sec: number;
            nsec: number;
        }[];
        max_velocity: number;
        max_acceleration: number;
        constructor(values: {
            header: StdMsgs.Header;
            profile: string;
            position: number[];
            time_from_start: {
                sec: number;
                nsec: number;
            }[];
            max_velocity: number;
            max_acceleration: number;
        });
    }
    class PeriodicCmd extends ROSLIB.Message {
        header: StdMsgs.Header;
        profile: string;
        period: number;
        amplitude: number;
        offset: number;
        constructor(values: {
            header: StdMsgs.Header;
            profile: string;
            period: number;
            amplitude: number;
            offset: number;
        });
    }
    enum MasterStateType {
    }
    const MASTER_NOPOWER: MasterStateType;
    const MASTER_STANDBY: MasterStateType;
    const MASTER_ON: MasterStateType;
    const MASTER_OFF: MasterStateType;
    const MASTER_SHUTDOWN: MasterStateType;
    enum CircuitStateType {
    }
    const STATE_NOPOWER: CircuitStateType;
    const STATE_STANDBY: CircuitStateType;
    const STATE_PUMPING: CircuitStateType;
    const STATE_ON: CircuitStateType;
    const STATE_ENABLED: CircuitStateType;
    const STATE_DISABLED: CircuitStateType;
    class PowerBoardState extends ROSLIB.Message {
        header: StdMsgs.Header;
        name: string;
        serial_num: number;
        input_voltage: number;
        master_state: PR2Msgs.MasterStateType;
        circuit_state: PR2Msgs.CircuitStateType[];
        circuit_voltage: number[];
        run_stop: boolean;
        wireless_stop: boolean;
        constructor(values: {
            header: StdMsgs.Header;
            name: string;
            serial_num: number;
            input_voltage: number;
            master_state: PR2Msgs.MasterStateType;
            circuit_state: PR2Msgs.CircuitStateType[];
            circuit_voltage: number[];
            run_stop: boolean;
            wireless_stop: boolean;
        });
    }
    class PowerState extends ROSLIB.Message {
        header: StdMsgs.Header;
        power_consumption: number;
        time_remaining: {
            sec: number;
            nsec: number;
        };
        prediction_method: string;
        relative_capacity: number;
        AC_present: number;
        constructor(values: {
            header: StdMsgs.Header;
            power_consumption: number;
            time_remaining: {
                sec: number;
                nsec: number;
            };
            prediction_method: string;
            relative_capacity: number;
            AC_present: number;
        });
    }
    class PressureState extends ROSLIB.Message {
        header: StdMsgs.Header;
        l_finger_tip: number[];
        r_finger_tip: number[];
        constructor(values: {
            header: StdMsgs.Header;
            l_finger_tip: number[];
            r_finger_tip: number[];
        });
    }
    class SetLaserTrajCmdRequest extends ROSLIB.ServiceRequest {
        command: PR2Msgs.LaserTrajCmd;
        constructor(values: {
            command: PR2Msgs.LaserTrajCmd;
        });
    }
    class SetLaserTrajCmdResponse extends ROSLIB.ServiceResponse {
        start_time: {
            sec: number;
            nsec: number;
        };
        constructor(values: {
            start_time: {
                sec: number;
                nsec: number;
            };
        });
    }
    class SetPeriodicCmdRequest extends ROSLIB.ServiceRequest {
        command: PR2Msgs.PeriodicCmd;
        constructor(values: {
            command: PR2Msgs.PeriodicCmd;
        });
    }
    class SetPeriodicCmdResponse extends ROSLIB.ServiceResponse {
        start_time: {
            sec: number;
            nsec: number;
        };
        constructor(values: {
            start_time: {
                sec: number;
                nsec: number;
            };
        });
    }
}
declare module 'PR2Msgs' {
    export = PR2Msgs;
}
