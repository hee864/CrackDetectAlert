# 🔍 Multi-Robot Collaborative System (YOLO + ROS2 + Navigation)

이 프로젝트는 **TurtleBot4 기반 협력 로봇 시스템**으로, 카메라 기반 객체 탐지와 네비게이션을 연동하여 **균열 감지, 사람 추적, 위험 판단 및 유도** 기능을 수행합니다.

---

## 📁 시스템 구성

총 4개의 주요 노드로 구성되며, 각 로봇의 역할을 명확히 분리했습니다.

| 노드 이름 | 설명 | 대상 로봇 |
|-----------|------|------------|
| `YOLOImageSubscriber` | YOLO 기반 객체 인식 + 좌표 변환 및 퍼블리시 | 로봇1 |
| `ObstacleFollower` | `/obstacle` 구독 → car(사람) 위치로 이동 → `/person_follow` 서비스 호출 | 로봇1 |
| `Robot2Dectctor` | 위험(crack) 발생 시 `/detected_human` 기반 출구 판단 | 로봇2 |
| `YOLO(Robot2)` | 로봇2에서도 YOLO로 위험 및 사람 위치 판단 → `/danger`, `/obstacle` 발행 | 로봇2 |

---

## 🔁 주요 토픽/서비스 흐름

### 📡 사용 토픽

- `/obstacle` : car(사람) 또는 crack(균열) 감지 결과
- `/danger` : crack 크기 기반 위험 감지 (위험=1.0, 안전=0.0)
- `/detected_human` : 5초간 유효한 car(사람) 위치 정보

### 🧩 사용 서비스

- `/person_follow` : 사람이 감지되어 따라갔음을 외부에 알림
- `/reperson_follow` : 재탐색 요청 (즉시 True 반환)

---

## ⚙️ 작동 시나리오

1. **YOLOImageSubscriber (로봇1)**  
   - RGB + Depth 영상 처리 → YOLO 감지 결과 분석  
   - car 감지 → `/detected_human` 발행  
   - crack 감지 → 균열 길이 계산 후 `/danger` 발행  

2. **ObstacleFollower (로봇1)**  
   - `/obstacle`에서 car 감지 → 해당 위치로 네비게이션 이동  
   - 이동 성공 시 `/person_follow` 서비스 호출  

3. **Robot2Dectctor (로봇2)**  
   - `/danger`에서 danger_sense=1.0 수신 시 추적 시작  
   - `/detected_human` 구독 → 가장 가까운 출구 판단 (exit4 or exit5)

---

## 🧪 실행 방법

### 1. YOLO 모델 준비

```bash
# YOLO 모델 파일 준비
/path/to/model/my_best.pt
