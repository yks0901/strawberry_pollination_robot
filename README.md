
# 💡 작품 소개영상


[![한이음 드림업 프로젝트 소개](https://raw.githubusercontent.com/yks0901/2025_/main/asset/%EC%8D%B8%EB%84%A4%EC%9D%BC.png)](https://youtu.be/DfwWIk6EnHk)
-----

# 🧠 Strawberry Pollination Robot

YOLO 객체 인식과 SLAM을 결합하여 딸기꽃을 인식하고  
매니퓰레이터로 자동 수분을 수행하는 자율 로봇 시스템  

---

## 🔥 My Contributions

- YOLOv8 기반 딸기꽃 객체 인식 모델 구현  
- Depth 카메라 기반 3D 위치 추정 (핀홀 카메라 모델 + 왜곡 보정)  
- TF를 활용한 camera → map → base 좌표 변환 파이프라인 구축  
- RTAB-Map 기반 Visual SLAM 적용 (자율주행 및 위치 추정)  
- MoveIt 기반 매니퓰레이터 IK / motion planning / execution 구현  

---

## 🏗 System Architecture

Camera → YOLO → 3D 좌표 → TF 변환 → MoveIt → Manipulation  

<img width="500" src="https://github.com/yks0901/2025_/blob/main/asset/moveit_flow" />
<img width="500" src="https://github.com/yks0901/2025_/blob/main/asset/%EC%84%BC%EC%84%9C%20%ED%9D%90%EB%A6%84%EB%8F%84.png" />

<img width="500" src="https://github.com/yks0901/2025_/blob/main/asset/top2.png" />
<img width="500" src="https://github.com/yks0901/2025_/blob/main/asset/bottom2.png" />

<img width="600" src="https://github.com/yks0901/2025_/blob/main/asset/algorithm_flow" />


---

## 🏆 Results

- 딥러닝 기반 딸기꽃 인식 및 자동 수분 동작 성공  
- Depth 기반 3D 좌표 추정 → TF 변환 → MoveIt 제어까지 전체 파이프라인 구현  
- 실제 환경에서 자율주행 및 매니퓰레이션 동작 검증  
- 딸기꽃 약 1m 이내 거리에서 안정적인 인식 및 접근 수행  
- 한이음 드림업 프로젝트 은상 수상  

---



