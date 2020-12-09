# raspberry pi를 장착하지 않고 PX4_드론을 원하는 위치에 옮기기
본 내용은 net챌린지 캠프 시즌7에 참여하면서 알게된 내용은 정리한 것으로, 맨땅 헤딩을 하며 얻은 것이다.
필자는 대학교 3학년이며 개발 능력이 매우 낮아 코드는 볼품없지만, 추후 다른 대학생이 드론프로젝트를 할 때 조금이나마 도움이 되고자 문서를 작성한다.
코드를 보고 충분히 원하는데로 수정가능한 수준이다.

# 프로젝트 시나리오(무시해도 됨)
1. 프로젝트의 서브 목표는 여러 사용자의 gps정보를 얻어와 k means clustering 알고리즘을 사용해 n개의 포인트를 얻는 것이다.

2. 필자는 시연용으로서, 랜덤으로 1000개의 포인트를 얻어와 clustering.py를 사용하여 2개의 지점을 얻어 파일로 저장했다.
  각 파일은 drone1.txt, drone2.txt로, 위도, 경도의 값이 저장되어 있다.

3. goToLocation.py, goToLocation2.py를 실행하면 파일의 값을 읽어 드론을 해당 위치로 보낸다.

4. 사용자들의 위치가 변했다고 가정하고 clustering.py를 다시 실행해 새로운 값을 얻어오면 드론은 자동으로 해당 위치에 이동한다.

# 개발 환경
OS: window10 education -> vmware workstation 16 -> Ubuntu 18.04
Drone frame: Holybro S500 v2
Pxhawk version: 4

# 사전 준비
1. python 설치
2. pip install Pymavlink
3. pip install mavproxy
4. pixhawk보드와 sik radio 펌웨어 업데이트

# 사용법
1. sik radio를 usb포트에 연결

2. 터미널에 dmesg를 입력하면 디바이스 연결 관련 로그 메세지가 출력되는데 거의 맨 밑에 방금 연결한 포트의 이름이 뜬다.
  나의 경우 /dev/ttyUSB0로 떴다.

3. mavproxy가 설치된 디렉토리로 이동하여 명령어 실행(ex. /usr/local/bin/mavproxy.py)

    sudo mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out udp:127.0.0.1:14540

  여기서는 ttyUSB0를 localhost의 14540번 포트에 연결함을 의미하고, baudrate는 전송속도와 관련된 값이다.
  얻은 포트번호가 /dev/ttyUSB0와 다르다면 저 부분만 바꾸면 된다.

  extra) QgroundControl은 드론을 제어하는 GUI 프로그램으로, 14550포트를 이용한다. 
  따라서 이론적으로 sudo mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out udp:127.0.0.1:14540 –-out udp:127.0.0.1:14550 명령을 입력하면 GCS에 udp로 연결할 수 있지만,
  필자는 불가능 했다

4. 여러 설정을 위해 goToLocation.py을 편집기로 들어간다.
  그 후, connection_string = '0.0.0.0:14540' 로 바꿔 방금 설장한 14540포트에 연결한다.
  필자는 고도(altitude)를 15m로 고정하여 날렸다. 필요하면 수정하면 된다.
  
5. 필요한 대로 수정을 완료 후 goToLocation.py를 실행하면 drone1.txt 파일을 읽어와 해당 위치로 보낸다.
  나는 clustering.py를 이용해 해당 파일을 만들어 냈으므로 필요 시 직접 파일을 만들거나 위치를 생성하는 코드를 만든다.
  drone1.txt의 첫째줄에는 위도, 둘째줄에는 경도 값이 들어가며 구글 지도에서 특정 위치를 클릭하면 나오는 값이다.
  
# 여러 드론 날리기
# 주의 사항
사용하는 sik radio가 AT 기능이 있거나 netID를 변경 가능한 radio여야 한다. 필자는 holybro telemetry radio 915MHZ를 이용했었는데, 해당 모듈은 AT기능을 지원하지 않아 라디오들의 통신이 간섭을 받아 제대로 작동하지 않았다. 반드시 주파수 조정이 가능한 라디오를 준비한다.

# 사용법
1. 거의 동일하다. 우선 하나의 드론을 완벽히 연결상태로 만든다.

2. 라디오를 usb포트에 연결 후 dmesg를 입력해 포트번호를 알아낸다.

3. 아까와 달리 mavproxy 이용 시 포트번호를 다르게 써야 한다. 예를 들면

   sudo mavproxy.py --master=/dev/ttyUSB0 --baudrate 57600 --out udp:127.0.0.1:14541
  
  로 사용하면 된다.
  
4. goToLocation.py의 복제본을 만든 뒤 connection_string = '0.0.0.0:14541'로 변경한다.

5. 받아올 텍스트파일 이름을 정해줘야 한다. 맨 밑의 while문을 보면 open('drone1.txt','r')로 되있으므로 알맞게 수정한다.

6. 사용한다.
