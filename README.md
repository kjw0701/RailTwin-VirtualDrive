# RailTwin 가상주행

HP028_RailTwin 대차 CAD에서 변환한 차체와 바퀴, 25 mm 각파이프 STL을 사용한다. 가운데 레일을 2 mm 높인 조건에서 거리와 가속도를 함께 기록한다.

MATLAB R2026a, Simulink, Simscape, Simscape Multibody가 있는 PC에서 저장소 루트를 열고 실행한다.

```matlab
cd matlab
run_paper_pipe(true)
```

결과는 matlab/output/paper_pipe_step_2mm_csvfit에 저장된다. true는 실측 정상 31~33차에 맞춘 가상 거리 센서 높이를 사용한다. 기본 3.5 mm 간격과 주행 영상은 run_paper_pipe()로 생성한다. 가속도 응답의 실측 일치도는 미검증이다.

## 가속도 모델을 수정하는 위치

레일 치수와 각 파이프의 시작·끝·윗면 높이는 `matlab/PaperRail.m`에서 관리한다. 같은 형상을 바퀴 접촉, 거리 센서, 화면 표시가 사용한다. Simulink에서 컴파일되는 `pipe_contact_kernel.m`은 가장 가까운 접촉점에서 강성·감쇠에 따른 힘을 구한다.

`matlab/ChassisIMU.m`에는 차체에 고정한 가속도계 한 개의 위치와 관측식이 있다. 좌우 센서 각각에 차체 병진 가속도, 회전에 의한 가속도, 중력을 적용한다. CSV의 L/R과 ax/ay/az를 조합한 여섯 개의 필터 출력 열(단위 m/s²)이 합성 IMU 입력이다. 좌우 거리의 CSV 보정은 거리 센서 위치에만 적용한다.

MATLAB의 matlab 폴더에서 `verify_pipe_physics`로 정지, 자유낙하, 등속 회전 및 바퀴 접촉의 기준 사례를 검사할 수 있다. 직선 주행과 요 회전은 구속되어 있고, 질량·접촉 강성·감쇠·센서 필터는 가정값이다. 이 검사는 운동 방정식과 구현을 점검한다. 가속도의 실제 충격 크기와 진동 주파수에 대한 실물 검증은 별도 과제다.

Python만 있는 PC에서는 저장소 루트에서 동봉한 합성 주행 1회를 읽을 수 있다.

```sh
python python/load_virtual_drive.py example
```

보정 주행을 실행한 뒤 `python python/check_reference_run.py`로 기존 예제와 같은 시각의 모든 열을 비교한다.
