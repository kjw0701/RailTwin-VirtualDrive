# RailTwin 가상주행

HP028_RailTwin 대차 CAD에서 변환한 차체와 바퀴, 25 mm 각파이프 STL을 사용한다. 가운데 레일을 2 mm 높인 조건에서 거리와 가속도를 함께 기록한다.

MATLAB R2026a, Simulink, Simscape, Simscape Multibody가 있는 PC에서 저장소 루트를 열고 실행한다.

```matlab
cd matlab
run_paper_pipe(true)
```

결과는 matlab/output/paper_pipe_step_2mm_csvfit에 저장된다. true는 실측 정상 31~33차에 맞춘 가상 거리 센서 높이를 사용한다. 기본 3.5 mm 간격과 주행 영상은 run_paper_pipe()로 생성한다. 가속도 응답의 실측 일치도는 미검증이다.

Python만 있는 PC에서는 저장소 루트에서 동봉한 합성 주행 1회를 읽을 수 있다.

```sh
python python/load_virtual_drive.py example
```
