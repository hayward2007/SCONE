class Mode :
    __default_initial_position = [ 180 for _ in range(6) ];
    upper_initial_position = __default_initial_position;
    middle_initial_position = __default_initial_position;
    lower_initial_position = __default_initial_position;

    __default_speed = 30;
    safety_speed = __default_speed;
    walk_speed = __default_speed;
    drive_speed = __default_speed;
    climb_speed = __default_speed;

    def __init__(self) :
        self.upper_initial_position = Mode.upper_initial_position;
        self.middle_initial_position = Mode.middle_initial_position;
        self.lower_initial_position = Mode.lower_initial_position;

        self.safety_speed = Mode.safety_speed;
        self.walk_speed = Mode.walk_speed;
        self.drive_speed = Mode.drive_speed;
        self.climb_speed = Mode.climb_speed;