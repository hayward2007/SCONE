class Actuator :
    speed = 100;

    index = [i for i in range(1, 19)];
    upper_right_index = [1, 4, 5];
    upper_left_index = [2, 3, 6];
    middle_right_index = [i + 6 for i in upper_right_index];
    middle_left_index = [i + 6 for i in upper_left_index];

    class position :
        start = 0;
        center = 2048;
        end = 4096;