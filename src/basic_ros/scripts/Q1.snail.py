#!/usr/bin/env python3
from time import *

"""
1번 문제.
달팽이가 100m 우물에 빠졌다. 달팽이는
낮에는 위로 5m 올라가고, 밤에는 밑으로 3m 미끄러질 때
며칠 만에 우물을 탈출할 수 있을까?

2번 문제.
작성한 코드를 1줄로 작성.
"""

up = 5
down = 3
snail = 0
distance = 100
day = 0

# Answer 1

start_time_1 = time()
while True:
    day += 1
    snail += up
    if snail > distance:
        break
    else:
        snail -= down
end_time_1 = time()
print(f"day:{day}")
print(f"time:{end_time_1 - start_time_1}")

# Answer 2
start_time_2 = time()
day = (distance - down) // (up - down) + 1
end_time_2 = time()
print(f"time:{end_time_2 - start_time_2}")
