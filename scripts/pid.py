from math import pi


class PID:
    KP = 0.5
    KD = 0.001
    KI = 0.1

    def __init__(self):
        self.err_integral = float()
        self.err_derivative = float()
        self.err = float()
        self.err_prev = float()

    def __set(self, err, dt):
        # type: (float, float) -> None

        self.err_prev = self.err
        self.err = err
        self.err_derivative = (self.err - self.err_prev) / dt
        self.err_integral += self.err * dt

    @property
    def __output_P(self):
        # type: () -> float
        return self.err * PID.KP  # type: ignore

    @property
    def __output_I(self):
        # type: () -> float
        # return self.err_integral * PID.KI

        return 0

    @property
    def __output_D(self):
        # type: () -> float
        return min(self.err_derivative * PID.KD, 0)

    def output(self, err, dt):
        #type: (float, float) -> float
        self.__set(err, dt)
        out = min(max(self.__output_P + self.__output_I + self.__output_D, 0),
                  pi / 10)

        return out