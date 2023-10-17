#!/usr/bin/env python3
from random import *

"""
1번 문제.
0에서 1000사이 랜덤한 숫자를 넣었을 때,
250 이하는 250으로,
250에서 750 사이는 입력한 숫자 그래도,
750 이상은 750으로,
출력하라

2번 문제.
작성한 코드를 1줄로 작성.
"""

number = randint(0, 1001)
print(f"input number : {number}")

# Answer 1
if number <= 250:
    number = 250
elif number >= 750:
    number = 750
else:
    pass
print(f"output number(A1) : {number}")

# Answer 2
number = randint(0, 1001)
num = sorted([250, number, 750])[1]
print(f"output number(A2) : {number}")
