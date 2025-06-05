from machine import Pin
from time import sleep_us, ticks_us, ticks_diff


print("Fichier stepper_control bien importé")

class StepperCommand:
    """The StepperCommand class offers a way to synchronize the movements of multiple stepper motors.
    The movements are first set with the prepare() method then executed with the run() method.
    """
    class _Command:
        # The _Command class holds the prepared commands keeping account of the stepper drivers and
        # the movement details.
        # This class is probably useless outside of the StepperCommand class, so it's private.
        def __init__(self, stepper, count, direction, full_steps):
            self._stepper = stepper
            self._count = count
            self._direction = direction
            self._full_steps = full_steps

            self._last_call_time = 0
            self._delay_before_next_step = 0
            self._delay_us = stepper.delay_us

        @property
        def delay_us(self):
            return self._delay_us

        @delay_us.setter
        def delay_us(self, us):
            self._delay_us = us

        def shortest_duration(self):
            return self._count * self._stepper.delay_us

        def spread(self, total_duration):
            self.delay_us = total_duration // self._count

        def __call__(self, current_time):
            self._delay_before_next_step -= current_time - self._last_call_time
            self._last_call_time = current_time
            if self._delay_before_next_step <= 0:
                # Move the stepper motor for one step without waiting for the motor to rest
                self._stepper.step(1, self._direction, self._full_steps, no_delay=True)
                self._delay_before_next_step = self.delay_us

    def __init__(self):
        """Initialize the StepperCommand instance.
        """
        self._commands = []

    def prepare(self, stepper, count=1, direction=1, full_steps=True):
        """The prepare() method stores the details of the movements to achieve.
        Important notice: preparing multiple commands on the same stepper may lead to unexpected results.

        :return: None
        """
        self._commands.append(StepperCommand._Command(stepper, count, direction, full_steps))

    def run(self):
        """The run() method executes the prepared stepper moves in parallel.

        :return: None
        """
        if len(self._commands) > 0:
            # The prepared mouvements will be run interleaved
            # Compute how they are interleaved
            # Get the longest movement
            total_duration = 0
            # The delays_us list will store the various prepared stepper delays.
            # It will then be used to compute the time resolution of the parallel moves.
            delays_us = []
            for s in self._commands:
                total_duration = max(total_duration, s.shortest_duration())
                delays_us.append(s.delay_us)

            # We need a proper gcd() function as Micropython doesn't provide any
            def gcd(x, *args):
                d = abs(x)
                for y in args:
                    y = abs(y)
                    while y:
                        d, y = y, d % y
                return d

            # Compute the time resolution of the moves
            time_grain = gcd(*delays_us)

            # Each stepper needs some rest time to set its position but moving 2 steppers in parallel
            # only need 1 rest.
            # s = step
            # _ = mandatory rest (could be different for each stepper motor)
            # . = pause
            # There are various ways to spread the motor steps around the total duration,
            # try this one.
            # stepper 1 with 10 steps: s_s_s_s_s_s_s_s_s_s_
            # stepper 2 with 5 steps:  s_..s_..s_..s_..s_..
            # stepper 3 with 3 steps:  s_.....s_.....s_....
            for s in self._commands:
                s.spread(total_duration)

            tick_start = ticks_us()
            while True:
                elapsed = ticks_diff(ticks_us(), tick_start)
                for s in self._commands:
                    s(elapsed)
                if elapsed >= total_duration:
                    break

            # Once the moves are executed, clear the prepared commands.
            self._commands.clear()

class ULN2003:
    """
    This class was written with the help of the information found in:
    http://www.jangeox.be/2013/10/stepper-motor-28byj-48_25.html
    """
    _HALF_STEPS = ((1, 0, 0, 1), (0, 0, 0, 1), (0, 0, 1, 1), (0, 0, 1, 0),
                   (0, 1, 1, 0), (0, 1, 0, 0), (1, 1, 0, 0), (1, 0, 0, 0))

    def __init__(self, pin0_id, pin1_id, pin2_id, pin3_id, delay_us=2000):
        """Initialize the ULN2003 instance.
        This expects 4 pin/gpio numbers in order to drive the ULN2003 chip.
        It also needs a duration in microsecond corresponding to the delay the motor needs to set
        its position between two steps. This depends on the kind of motor driven by the ULN2003,
        and on the characteristics of the motor power supply.

        :param pin0_id: int
        :param pin1_id: int
        :param pin2_id: int
        :param pin3_id: int
        :param delay_us: int
        """
        # Initialize the 4 pins
        # These could have been nicely stored in a list but then, they would have needed
        # to be indirectly accessed at each step.
        self._pin0 = Pin(pin0_id, Pin.OUT)
        self._pin1 = Pin(pin1_id, Pin.OUT)
        self._pin2 = Pin(pin2_id, Pin.OUT)
        self._pin3 = Pin(pin3_id, Pin.OUT)

        self._delay_us = delay_us

        # The _step private property keeps the index of the last output-pin combination
        # of the _HALF_STEPS table used by the step() method.
        self._step = 0

    @property
    def delay_us(self):
        """Returns the configured delay between two steps."""
        return self._delay_us

    def _set_pins(self, v0, v1, v2, v3):
        """The _set_pins() private method sets the pin states."""
        self._pin0.value(v0)
        self._pin1.value(v1)
        self._pin2.value(v2)
        self._pin3.value(v3)

    def disable(self):
        """The disable() method sets all coils to the ground, disabling the driven motor."""
        self._set_pins(0, 0, 0, 0)

    def step(self, count=1, direction=1, full_steps=True, no_delay=False):
        """The step() method moves the stepper by changing the motor coils state the right way
        to rotate the motor shaft.

        :param count: int, the number of steps
        :param direction: int, positive value makes the motor shaft to rotate in a direction,
        while negative value make it rotate in the opposite direction. If 0 the motor won't move.
        :param full_steps: bool, when False the steps have half the amplitude of when it's True.
        :param no_delay: bool, if True, the method will sleep for a preset (at instanciation) amount of time.
        This allows time for the motor to move. If False, the method won't sleep.
        :return: None
        """
        # constrain direction to be either -1, 0 or 1.
        direction = max(min(int(direction), 4), -4)
        print(direction)
        # Then for the number of steps proceed around the step table
        if full_steps:
            # skip half steps
            direction *= 2

        # Pausing after each step ensures the motor has time to move and set
        # If the method is called for a single step, the delay could be omitted (delegated to the caller).
        delay_us = self._delay_us
        if no_delay and count == 1:
            delay_us = 0

        for c in range(count):
            self._step = (self._step + direction + len(self._HALF_STEPS)) % len(self._HALF_STEPS)
            self._set_pins(*self._HALF_STEPS[self._step])
            sleep_us(delay_us)


class DRV8825:
    """
    This class is a work in progress.
    It is designed to control a DRV8825 stepper driver.
    """
    def __init__(self, step_pin_id, dir_pin_id, delay_us=2000):
        # Initialize the pins
        self._step_pin = Pin(step_pin_id, Pin.OUT)
        self._dir_pin = Pin(dir_pin_id, Pin.OUT)
        self._ledpin = Pin(25, Pin.OUT)
 
        self._delay_us = delay_us

    @property
    def delay_us(self):
        """Returns the configured delay between two steps."""
        return self._delay_us

    def step(self, count=1, direction=1, no_delay=False):
        """The step() method moves the stepper.

        :param count: int, the number of steps
        :param direction: int, positive value makes the motor shaft to rotate in a direction,
        while negative value make it rotate in the opposite direction. If 0 the motor won't move.
        :param no_delay: bool, if True, the method will sleep for a preset (at instanciation) amount of time.
        This allows time for the motor to move. If False, the method won't sleep.
        :return: None
        """
        if direction != 0:
            # (if direction == 0 the motor won't move)
            # Set the dir pin
            self._dir_pin.value(1 if direction > 0 else 0)

            # Pausing after each step ensures the motor has time to move and set
            # If the method is called for a single step, the delay could be omitted (delegated to the caller).
            delay_us = self._delay_us
            if no_delay and count == 1:
                delay_us = 0

            # Pulse the step pin for each step
            # The DRV8825 datasheet says the maximum frequency of the step pin is 250kHz
            # and the minimum duration of a step pulse is 1.9 µs.
            for c in range(count):
                self._step_pin.value(1)
                self._ledpin.value(1) 
                sleep_us(1000)  # 2 µs pause
                self._step_pin.value(0)
                self._ledpin.value(0)
                #print(c, "pulse")
                sleep_us(max(0, delay_us))
                #print(max(0, delay_us))


if __name__ == "__main__":
    # Démonstration de l'utilisation de la classe ULN2003.
    from machine import reset

    def uln2003_demo():
        # Initialize the ULN2003 class in order to drive a motor connected through
        # GPIOs 10 to 13:
        #   Pico        ULN2003
        #   GPIO_13 --> IN_1
        #   GPIO_12 --> IN_2
        #   GPIO_11 --> IN_3
        #   GPIO_10 --> IN_4
        stepper_driver = ULN2003(13, 12, 11, 10)
        # By default, the step delay is set to 2000µs this may need to be adjusted
        # depending on the steppers power supply and the type of steppers used.
        # stepper_driver = ULN2003(13, 12, 11, 10, delay_us=1900)

        # Number of steps to do below.
        steps = 500
        # Move in a direction in full steps then backward.
        # Doing full steps is the default but it could be changed.
        stepper_driver.step(steps, 1)
        stepper_driver.step(steps, -1)
        # Do half steps moves.
        stepper_driver.step(steps, 1, full_steps=False)
        stepper_driver.step(steps, -1, full_steps=False)

        # Disabling the motor stops power consumption but releases the motor from holding its position,
        # take care if the motor is under load.
        stepper_driver.disable()

        # In order to synchronize the moves of multiple stepper motors, we'll use a StepperCommand object
        stepper_command = StepperCommand()
        # Once the class instance has been created, prepare the wanted moves by calling de prepare() method
        # with the stepper driver, the number of steps to move, the direction and if steps will be full or half.
        stepper_command.prepare(stepper_driver, 500)
        # Let say we have a second stepper motor, a bit faster than the first.
        other_stepper_driver = ULN2003(21, 20, 19, 18, delay_us=1900)
        # We will to twice more steps but they will be half steps and reversed.
        stepper_command.prepare(other_stepper_driver, count=500, direction=-1, full_steps=False)
        # Then execute the prepared commands
        stepper_command.run()

    def drv8825_demo():
        # Initialize the DRV8825 class in order to drive a motor connected through
        # GPIOs 10 to 13:
        #   Pico        DRV8825
        #   GPIO_16 --> STEP
        #   GPIO_17 --> DIR
        stepper_driver = DRV8825(16, 17, 2000)
        # By default, the step delay is set to 2000µs this may need to be adjusted
        # depending on the steppers power supply and the type of steppers used.

        # Number of steps to do
        print("avant les pas")
        steps = 1
        # Move in a direction in full steps then backward.
        # Doing full steps is the default but it could be changed.
        # c'est ici et pas plus haut que les steps avancent ou reculent (dans le driver).
        stepper_driver.step(steps, 1)
        #stepper_driver.step(10, 1)
#         stepper_driver.step(10, 1)
#         stepper_driver.step(10, -1)


    # Le programme ci-dessous est inclus dans un gestionnaire d'exception afin de s'arrêter proprement
    # s'il est interrompu.
    try:
        # Uncomment on of the following lines
        #uln2003_demo()
        drv8825_demo()
    except KeyboardInterrupt:
        # L'utilisateur a interrompu le programme, on réinitialise la carte.
        reset()
