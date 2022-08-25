"""
This is where we will store fast-math and general usage enhancement code.
"""

from ulab import numpy as np  # noqa


def arry_from_ints(*args):
    """
    Makes returns a tuple of np arrays
    """
    return np.array(args)


def ints_from_arry(arry):
    """
    Takes the members of the array and returns them as ints / floats... whatever.
    """
    result = list()
    for mem in arry:
        result.append(mem[0])
    return result


class MathWrap:
    """
    let's see if this works...
    """
    @staticmethod
    def add(a, b):
        """
        np add
        """
        return np.sum(
            arry_from_ints(
                a,
                b
            )
        )

    @staticmethod
    def subtract(a, b):
        """
        np subtract
        """
        return np.sum(
            arry_from_ints(
                a,
                -b
            )
        )

    @staticmethod
    def multiply(a, b):
        """
        np multiply
        """
        answer = np.array([a]) * np.array([b])
        return answer[0]

    @staticmethod
    def divide(a, b):
        """
        np divide
        """
        answer = np.array([a]) / np.array([b])
        return answer[0]


mw = MathWrap()
