수정 내용

1. Waste 동작을 두 부분으로 나누어 구현 
  - MWRD : 버리는 대기 위치로 이동 
  - MWPR : 버리는 동작 후  Load 위치로 이동 
           현재 위치가 MWRD로 이동한 위치가 아니면 
           E14 에러 코드 리턴 
  * POINT 추가 
    Point 11 => Y축. 버리는 대기 위치로 사용 

2. AWAS 동작 : SCARA와 I/O 신호를 사용해서 동기화 동작 
  - 버리는 대기 위치로 이동 후 Output On 
  - SCARA 신호를 대기하고, SCARA가 신호를 주면 바로 나머지 동작 실행 
  
  * POINT 추가 
    Poiint 11 => Y축. 버리는 대기 위치로 이동 
  * VAR 추가 
    V13 => SCARA 동기 신호를 받는 Input 번호 (V13=15) 
    V14 => SCARA 동기 신호 Timeout 시간 (V14=5000) 
    V15 => SCARA 동기 신호 사용여부 (V15=0 사용. V15=1 미사용) 

테스트 하기 전, 아래 내용 입력 
- Point Data 11번에 "0,90,0" 입력 
- VAR 13=15 
  VAR 14=5000
  VAR 15=0

 