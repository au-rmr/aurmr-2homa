"""A fixed sequence of cap board samples covering rest, even load, one-sided load, release with
a leftover offset, overload and light load. Plain Python so the old ROS 1 image (Python 3.8,
numpy 1.17) and the new image make exactly the same numbers."""
import random


def samples():
    rng = random.Random(20261107)
    base = [3_000_000 + 100_000 * k for k in range(8)]
    out = []
    for i in range(600):
        if i < 100:
            load = [0] * 8
        elif i < 200:
            load = [min(40_000, 400 * (i - 100))] * 8
        elif i < 300:
            load = [80_000, 80_000, 80_000, 5_000, 5_000, 5_000, 5_000, 20_000]
        elif i < 350:
            load = [3_000] * 8
        elif i < 400:
            load = [150_000] * 8
        elif i < 450:
            load = [150_000, 1_200_000, 1_200_000, 150_000, 150_000, 150_000, 150_000, 150_000]
        else:
            load = [2_000, 2_500, 1_500, 2_000, 2_200, 1_800, 2_000, 2_100]
        values = [b + l + rng.randint(-300, 300) for b, l in zip(base, load)]
        out.append((1_000_000 + 20 * i, values))   # device time in ms, 50 Hz
    return out
