class Actuator :
    # safety_speed = 50;
    # walking_speed = 100;
    # driving_speed = 200;

    index = [i for i in range(1, 19)];
    upper_index = [i for i in range(1, 7)];
    upper_right_index = [1, 3, 5];
    upper_left_index = [2, 4, 6];
    middle_index = [i for i in range(7, 13)];
    middle_right_index = [i + 6 for i in upper_right_index];
    middle_left_index = [i + 6 for i in upper_left_index];
    lower_index = [i for i in range(13, 19)];
    lower_right_index = [i + 6 for i in middle_right_index];
    lower_left_index = [i + 6 for i in middle_left_index];

    upper_diagonal_right_index = [1, 4, 5];
    upper_diagonal_left_index = [2, 3, 6];
    middle_diagonal_right_index = [i + 6 for i in upper_diagonal_right_index];
    middle_diagonal_left_index = [i + 6 for i in upper_diagonal_left_index];
    lower_diagonal_right_index = [i + 6 for i in middle_diagonal_right_index];
    lower_diagonal_left_index = [i + 6 for i in middle_diagonal_left_index];

    class torque :
        off = 0;
        on = 1;

    class position :
        start = 0;
        center = 2048;
        end = 4096;
    
    class model :
        class MX :
            protocol_version = 1.0;
            class address :
                present_position = 36;
                goal_position = 30;
                enable_torque = 24;
                moving_speed = 32;
        
        class XM :
            protocol_version = 2.0;
            class address :
                profile_acceleration = 108;
                present_position = 132;
                present_velocity = 128;
                profile_velocity = 112;
                operating_mode = 11;
                goal_position = 116;
                goal_velocity = 104;
                torque_enable = 64;

            class operating_mode :
                velocity = 1;
                position = 3;
                extended_position = 4;
