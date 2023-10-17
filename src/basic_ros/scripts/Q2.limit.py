#!/usr/bin/env python3
from random import *  # random 모듈을 가져옵니다.

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

# 0에서 1000 사이의 랜덤한 숫자를 생성하고 출력합니다.
number = randint(0, 1001)
print(f"input number : {number}")

# Answer 1
# 아래는 랜덤한 숫자를 받아서 특정 범위에 따라 값을 수정하는 코드입니다.

if number <= 250:  # 만약 숫자가 250 이하라면
    answer = 250  # 숫자를 250으로 수정
elif number >= 750:  # 만약 숫자가 750 이상이라면
    answer = 750  # 숫자를 750으로 수정
else:  # 그 외의 경우에는 (250 초과 750 미만)
    answer = number  # 숫자를 그대로 둠
print(f"output number(A1) : {answer}")  # 결과를 출력

# Answer 2 - 1
# 아래는 랜덤한 숫자를 받아서 특정 범위에 따라 값을 수정하는 코드를 1줄로 작성한 것입니다.
answer = max(250, min(number, 750))
print(f"output number(A2-1) : {answer}")  # 결과를 출력

# Answer 2 - 2
# 아래는 랜덤한 숫자를 받아서 특정 범위에 따라 값을 수정하는 코드를 1줄로 작성한 다른 버전입니다.
answer = sorted([250, number, 750])[1]
print(f"output number(A2-2) : {answer}")  # 결과를 출력
