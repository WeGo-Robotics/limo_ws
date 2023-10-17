#!/usr/bin/env python3
from time import *  # time 모듈을 가져옵니다.

"""
1번 문제.
달팽이가 100m 우물에 빠졌다. 달팽이는
낮에는 위로 5m 올라가고, 밤에는 밑으로 3m 미끄러질 때
며칠 만에 우물을 탈출할 수 있을까?

2번 문제.
작성한 코드를 1줄로 작성.
"""

# Answer 1
# 아래는 달팽이가 100m 우물에서 탈출하는데 걸리는 시간을 계산하는 코드입니다.

up = 5  # 낮에 위로 올라가는 높이
down = 3  # 밤에 미끄러지는 높이
snail = 0  # 달팽이의 현재 위치
distance = 100  # 우물의 높이
day = 0  # 걸린 날짜

start_time_1 = time()  # 코드 실행 시작 시간 기록

while True:
    day += 1  # 하루가 지남
    snail += up  # 낮에 올라감
    if snail > distance:  # 우물을 넘어가면
        break  # 반복문 종료
    else:
        snail -= down  # 밤에 미끄러짐

end_time_1 = time()  # 코드 실행 종료 시간 기록
print(f"day:{day}")  # 걸린 날짜 출력
print(f"time:{end_time_1 - start_time_1}")  # 코드 실행 시간 출력

# Answer 2
# 아래는 달팽이가 100m 우물에서 탈출하는데 걸리는 시간을 계산하는 코드를 1줄로 작성한 것입니다.
start_time_2 = time()
day = (distance - down) // (up - down) + 1
end_time_2 = time()
print(f"time:{end_time_2 - start_time_2}")  # 코드 실행 시간 출력
