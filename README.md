# Project-SmartCar

## 프로젝트 소개
전방 장애물 거리에 따라 차량 속도를 자동 제어, 속도·거리 등 운행 정보를 서버 DB에 실시간 전송·저장하는 전방 안전 제어 시스템 개발
<br>

## 개발 기간
* 25.08.20 - 25.08.26

### 개발 환경
- `Raspberry Pi 4`
- `Arduino Uno`
- `STM32`
- `MariaDB`

## 주요 기능
#### 전방 장애물 거리에 따른 모터 제어
- 10cm 미만 -> Motor PWM 0%
- 10-30cm  -> Motor PWM 거리에 따른 비례 제어
- 30cm 초과 -> Motor PWM 100%

#### TCP/IP 소켓 통신을 이용한 DB 데이터 전송
- Wi-Fi / Bluetooth 인터페이스로 디바이스 데이터를 전송받아 MariaDB에 자동 기록하는 데이터 파이프라인 구현
- 모터 PWM 설정값, 초음파 센서 기반 전방 장애물 거리, 엔코더 기반 실제 속도(RPM)를 측정하여 데이터로 저장

<img width="1070" height="595" alt="스크린샷 2025-11-27 오전 11 42 16" src="https://github.com/user-attachments/assets/dae6b422-00a9-4d3f-8eda-5b1156e2668d" />
