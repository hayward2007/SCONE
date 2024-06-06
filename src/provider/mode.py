class Mode :
    __default_upper_position = [ 180 for _ in range(6) ];
    __default_middle_position = 180;
    __default_lower_position = 180;

    __default_speed = 30;

    def __init__(self) :
        self.upper_initial_position = self.__default_upper_position;
        self.middle_initial_position = self.__default_middle_position;
        self.lower_initial_position =self.__default_lower_position;

        self.safety_speed = self.__default_speed;
        self.walking_speed = self.__default_speed;
        self.driving_speed = self.__default_speed;
        self.climbing_speed = self.__default_speed;

        self.controller = None;