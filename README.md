# ESP32-S3 Drone Controller

## 한국어

### 프로젝트 소개

이 프로젝트는 **Seeed Studio XIAO ESP32-S3** 보드를 기반으로 제작한 무선 드론 컨트롤러 펌웨어입니다. PS2 호환 컨트롤러의 입력을 별도의 전용 라이브러리 없이 GPIO 비트뱅 방식으로 읽고, 입력값을 드론 제어 패킷으로 변환하여 ESP-NOW로 송수신합니다.

단순한 무선 조종 기능뿐 아니라 컨트롤러 입력 처리, 명령 데이터 구조화, 무선 패킷 직렬화·역직렬화 과정을 분리하여 각 기능을 독립적으로 관리할 수 있도록 구현했습니다.

### 개발 환경

| 항목 | 내용 |
| --- | --- |
| 개발 보드 | Seeed Studio XIAO ESP32-S3 |
| MCU | ESP32-S3 |
| 프레임워크 | ESP-IDF 6.0 |
| 언어 | C / C++ |
| 입력 장치 | PS2 호환 컨트롤러 |
| 무선 통신 | ESP-NOW |

### 주요 기능

#### 1. PS2 컨트롤러 비트뱅 인터페이스

- GPIO를 직접 제어하여 PS2 컨트롤러의 직렬 통신 인터페이스 구현
- 버튼과 아날로그 스틱 입력 데이터 수집
- 아날로그 스틱의 중심 오차와 미세 진동을 줄이기 위한 데드밴드 처리
- 조종 감도를 조절하기 위한 입력 곡선 및 필터 적용
- 읽은 원시 데이터를 드론 제어에 사용하는 정규화된 값으로 변환

#### 2. ESP-NOW 송수신

- Wi-Fi 연결 과정 없이 컨트롤러와 드론 간 저지연 무선 통신 구현
- 조종 명령 패킷 송신 및 상태 데이터 수신 구조 구성
- 송신 성공 여부를 확인할 수 있는 콜백 기반 처리
- 일정 주기로 최신 조종 데이터를 전송하도록 통신 루프 구성

#### 3. 송수신 패킷 파서

- 컨트롤러 입력값을 무선 전송용 패킷으로 변환
- 수신 바이트 데이터를 정의된 구조체와 명령 값으로 복원
- 패킷 순서 확인을 위한 시퀀스 값 관리
- 조종 데이터와 Arm, 비상 정지, 비행 모드 등의 명령을 하나의 패킷으로 구성
- 컨트롤러 입력 처리와 ESP-NOW 통신 계층을 분리하여 유지보수성과 확장성 확보

### 입력 및 명령 구성

| 입력 | 드론 제어 기능 |
| --- | --- |
| Left Stick Y | Throttle |
| Left Stick X | Yaw |
| Right Stick X | Roll |
| Right Stick Y | Pitch |
| START | Arm / Disarm |
| SELECT | 비행 모드 변경 |
| L1 | 비상 정지 |

> 실제 버튼 매핑과 조종 방향은 펌웨어 설정에 따라 변경할 수 있습니다.

### 데이터 흐름

```mermaid
flowchart LR
    A[PS2 Controller] --> B[GPIO Bit-Bang Interface]
    B --> C[Input Processing]
    C --> D[Control Packet Parser]
    D --> E[ESP-NOW TX/RX]
    E <--> F[Drone Flight Controller]
```

1. 비트뱅 인터페이스가 PS2 컨트롤러의 버튼과 스틱 데이터를 읽습니다.
2. 입력 처리 단계에서 데드밴드, 입력 곡선, 저역 통과 필터 및 변화율 제한을 적용합니다.
3. 처리된 입력을 Roll, Pitch, Yaw, Throttle 및 상태 명령이 포함된 제어 패킷으로 변환합니다.
4. ESP-NOW를 통해 패킷을 드론으로 전송하고 필요한 상태 데이터를 수신합니다.

### 구현 시 고려 사항

- 비트뱅 통신의 클록과 데이터 타이밍이 다른 태스크의 실행에 의해 크게 흔들리지 않도록 구성해야 합니다.
- ESP-NOW 콜백에서는 오래 걸리는 연산을 피하고, 수신 데이터를 별도의 처리 구간으로 전달하는 방식이 적합합니다.
- 조종 패킷에는 시퀀스 번호와 수신 시간 정보를 사용하여 중복 패킷과 통신 타임아웃을 판별할 수 있습니다.
- Arm 및 비상 정지 명령은 일반 스틱 입력과 구분하여 예기치 않은 상태 변경을 방지해야 합니다.

### 빌드 및 실행

ESP-IDF 6.0 개발 환경에서 다음 명령으로 프로젝트를 빌드하고 다운로드할 수 있습니다.

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

빌드 전 PS2 컨트롤러 GPIO, ESP-NOW 피어 주소, Wi-Fi 채널 및 조종 입력 설정을 대상 하드웨어에 맞게 확인해야 합니다.

### 프로젝트 목적

- PS2 컨트롤러 프로토콜과 GPIO 비트뱅 통신 구현 경험 확보
- ESP-NOW 기반 실시간 제어 데이터 송수신 구조 검증
- 입력 장치, 패킷 파서 및 무선 통신 계층의 모듈화
- 직접 제작한 드론 비행 컨트롤러와 연동 가능한 전용 조종기 구현

---

## English

### Overview

This project implements a wireless drone controller using the **Seeed Studio XIAO ESP32-S3**. It reads a PS2-compatible controller through a custom GPIO bit-bang interface, converts the input into structured flight-control commands, and exchanges packets with the drone over ESP-NOW.

The firmware separates controller input handling, command processing, packet serialization, and wireless communication so that each part can be maintained and extended independently.

### Development Environment

| Item | Details |
| --- | --- |
| Development board | Seeed Studio XIAO ESP32-S3 |
| MCU | ESP32-S3 |
| Framework | ESP-IDF 6.0 |
| Languages | C / C++ |
| Input device | PS2-compatible controller |
| Wireless communication | ESP-NOW |

### Key Features

#### 1. PS2 Controller Bit-Bang Interface

- Implements the PS2 serial interface by directly controlling GPIO signals
- Reads button and analog-stick data
- Applies deadband processing to reduce center offset and small input fluctuations
- Supports input curves and filtering for adjustable control response
- Converts raw controller data into normalized flight-control values

#### 2. ESP-NOW Communication

- Provides low-latency communication between the controller and drone without a conventional Wi-Fi connection
- Transmits control-command packets and receives status data
- Uses callbacks to report transmission results
- Sends the latest controller state at a fixed update interval

#### 3. TX/RX Packet Parser

- Converts processed controller input into wireless packets
- Restores received byte data into defined structures and command values
- Manages sequence values for packet-order validation
- Combines flight-control data with commands such as arm, emergency stop, and flight-mode selection
- Separates input processing from ESP-NOW transport for improved maintainability and extensibility

### Input Mapping

| Input | Drone control function |
| --- | --- |
| Left Stick Y | Throttle |
| Left Stick X | Yaw |
| Right Stick X | Roll |
| Right Stick Y | Pitch |
| START | Arm / Disarm |
| SELECT | Change flight mode |
| L1 | Emergency stop |

> The button mapping and control directions can be changed in the firmware configuration.

### Data Flow

```mermaid
flowchart LR
    A[PS2 Controller] --> B[GPIO Bit-Bang Interface]
    B --> C[Input Processing]
    C --> D[Control Packet Parser]
    D --> E[ESP-NOW TX/RX]
    E <--> F[Drone Flight Controller]
```

1. The bit-bang interface reads button and analog-stick data from the PS2 controller.
2. Input processing applies deadband, response curves, low-pass filtering, and slew-rate limiting.
3. The processed values are converted into a packet containing roll, pitch, yaw, throttle, and state commands.
4. ESP-NOW sends the packet to the drone and receives the required status data.

### Implementation Considerations

- Bit-bang clock and data timing should be protected from excessive scheduling jitter caused by other tasks.
- ESP-NOW callbacks should remain short; received data should be forwarded to a separate processing context when additional work is required.
- Sequence numbers and reception timestamps can be used to detect duplicate packets and communication timeouts.
- Arm and emergency-stop commands should be handled separately from normal stick input to prevent unintended state changes.

### Build and Flash

Build and flash the project in an ESP-IDF 6.0 environment:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

Before building, verify the PS2 controller GPIO assignments, ESP-NOW peer address, Wi-Fi channel, and control-input configuration for the target hardware.

### Project Goals

- Implement and study the PS2 controller protocol through GPIO bit-banging
- Validate an ESP-NOW transport for real-time control data
- Modularize the input-device, packet-parser, and wireless-communication layers
- Build a dedicated transmitter compatible with a custom drone flight controller
