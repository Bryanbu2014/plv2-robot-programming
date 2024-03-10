from math import copysign, isclose
from numbers import Number


class Controller:
    def __init__(
        self,
        max_out: Number = 100,
        min_out: Number = 20,
        input_tolerance: Number = 0.05,
        output_max_if_error_is: Number = 1,
    ):
        self.max_out = max_out
        self.min_out = min_out
        self.input_tolerance = input_tolerance
        self.output_max_if_error_is = output_max_if_error_is

        # At `output_max_if_error_is` our output should be the maximum allowed
        # output (`max_out`). The controller either:
        # 1. starts decelerating
        # 2. stops accelerating (the output stays at `max_out`)
        # Calculate a gain value for the output so that the output is `max_out` if
        # the error is `output_max_if_error_is`.
        self._out_gain = max_out / output_max_if_error_is

    def out(self, error: Number = 0):
        if isclose(error, 0, abs_tol=self.input_tolerance):
            return 0
        else:
            error_abs = abs(error)
            error_sign = copysign(1, error)
            if error_abs > self.output_max_if_error_is:
                out_abs = self.max_out
            else:
                # Upper and lower bound
                out_abs_low_bounded = max(
                    [self.min_out, self._out_gain * error_abs]
                )
                out_abs_low_up_bounded = min(
                    [self.max_out, out_abs_low_bounded]
                )
                out_abs = out_abs_low_up_bounded
            return error_sign * out_abs

    def __call__(self, *args, **kwds):
        return self.out(*args, **kwds)
