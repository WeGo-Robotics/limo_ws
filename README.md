# limo_ws

1.터미널에 다음과 같이 입력하여 코드를 다운로드 받습니다.

git clone https://github.com/WeGo-Robotics/limo_ws.git 

2.다운로드 받은 limo_ws 폴더로 이동합니다.

cd limo_ws

3. 터미널에 다음과 같이 입력하여, 원클릭 셋팅 파일의 모든 권한을 열어줍니다.
sudo chmod 777 permission.bash



4. 터미널에 다음과 같이 입력하여 원클릭 셋팅 파일의 권한을 열어줍니다.

sudo chmod 777 permission.bash

5. 터미널에 다음과 같이 입력하여 원클릭 셋팅 파일을 실행합니다. 

./permission.bash

※원클릭 셋팅 파일(permission.sh)은 빌드(catkin_make) 및 사용하시는 쉘(bash or zsh)에 맞춰서
쉘의 환경설정 파일(~/.bashrc or ~/.zshrc)에 현재 워크스페이스($Current_path)의 경로를
설정(source devel/setup.bash or source devel/setup.zsh)합니다.



6. 터미널에 다음과 같이 출력되면 패스워드를 입력합니다.

password for $USER:

7.터미널에 다음과 같이 입력하여, ros master를 실행합니다.

roscore

8. roscore가 실행되었다면, 터미널에 다음과 같이 입력하여 파일을 실행해 봅니다.

ex)

rosrun opencv_edu 1.grayscale.py

9.rostopic list를 입력하여, 코드가 실행되어 있는지 확인합니다.

rostopic list
