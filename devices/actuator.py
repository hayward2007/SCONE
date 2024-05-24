class Actuator :
    speed = 50;

    index = [i for i in range(1, 19)];
    upper_right_index = [1, 3, 5];
    upper_left_index = [2, 4, 6];
    middle_right_index = [i + 6 for i in upper_right_index];
    middle_left_index = [i + 6 for i in upper_left_index];
    lower_right_index = [i + 6 for i in middle_right_index];
    lower_left_index = [i + 6 for i in middle_left_index];

    upper_diagonal_right_index = [1, 4, 5];
    upper_diagonal_left_index = [2, 3, 6];
    middle_diagonal_right_index = [i + 6 for i in upper_diagonal_right_index];
    middle_diagonal_left_index = [i + 6 for i in upper_diagonal_left_index];
    lower_diagonal_right_index = [i + 6 for i in middle_diagonal_right_index];
    lower_diagonal_left_index = [i + 6 for i in middle_diagonal_left_index];

    class mode :
        velocity_control = 1;
        position_control = 2;

    class position :
        start = 0;
        center = 2048;
        end = 4096;
    class model :
        class MX :
            protocol_version = 1.0;
            present_position = 36;
            goal_position = 30;
            enable_torque = 24;
            moving_speed = 32;
        
        class XM :
            protocol_version = 2.0;
            present_position = 132;
            present_velocity = 128;
            profile_velocity = 112;
            operating_mode = 11;
            goal_position = 116;
            goal_velocity = 104;
            torque_enable = 64;