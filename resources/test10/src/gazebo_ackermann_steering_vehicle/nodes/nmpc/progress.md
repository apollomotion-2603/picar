- logging prediction horizon parameter
- logging vehicle parameter
- checking signal velocity and steering angle
- logging and observing vehicle's states
    [measured states:
        longitudinal velocity
        lateral velocity
        angular velocity
    ]

    [parameter:
        length_to_front
        length_to_rear
    ]

    [sensors:
        camera
        IMU: ngoài tích phân phân Euler, cần kết hợp với quaternion
    ]

- create independent node for IMU
    - processing signal