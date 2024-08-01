#!/usr/bin/env python3

import decimal


class Pi:
    def __init__(self):

        # set context precsion
        decimal.DefaultContext.prec = 1000
        decimal.DefaultContext.rounding = decimal.ROUND_DOWN
        decimal.setcontext(decimal.DefaultContext)

        # calculate pi to set precision
        self.PI = self.calc_pi()

    def calc_pi(self):
        """Calculate pi to the given precision

        source: https://docs.python.org/3/library/decimal.html
        """

        decimal.getcontext().prec += 2

        three = decimal.Decimal(3)
        last_s, t, s, n, na, d, da = 0, three, 3, 1, 0, 0, 24

        while s != last_s:
            last_s = s
            n, na = n+na, na+8
            d, da = d+da, da+32
            t = (t*n)/d
            s += t

        decimal.getcontext().prec -= 2
        return +s
