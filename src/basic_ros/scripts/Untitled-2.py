from time import *

up = 5
down = 3
snail = 0
day = 0
distance = 1000000000000000000000000000000
start_time = time()
# while True:
#     day = day + 1
#     snail = snail + up
#     if snail >= distance:
#         break
#     else:
#         snail = snail - down


day = (distance - down) // (up - down) + 1
end_time = time()


print(end_time - start_time)
print(day)


# distance = day * up - (day - 1) * down
