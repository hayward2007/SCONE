class Mode :
    __default_initial_position = [ 180 for _ in range(6) ];
    # upper_initial_position = __default_initial_position;
    # middle_initial_position = __default_initial_position;
    # lower_initial_position = __default_initial_position;

    __default_speed = 30;
    # safety_speed = __default_speed;
    # walking_speed = __default_speed;
    # driving_speed = __default_speed;
    # climbing_speed = __default_speed;

    def __init__(self) :
        # self.upper_initial_position = Mode.upper_initial_position;
        # self.middle_initial_position = Mode.middle_initial_position;
        # self.lower_initial_position = Mode.lower_initial_position;

        self.upper_initial_position = self.__default_initial_position;
        self.middle_initial_position = self.__default_initial_position;
        self.lower_initial_position =self.__default_initial_position;

        # self.safety_speed = Mode.safety_speed;
        # self.walking_speed = Mode.walk_speed;
        # self.driving_speed = Mode.drive_speed;
        # self.climbing_speed = Mode.climb_speed;

        self.safety_speed = self.__default_speed;
        self.walking_speed = self.__default_speed;
        self.driving_speed = self.__default_speed;
        self.climbing_speed = self.__default_speed;