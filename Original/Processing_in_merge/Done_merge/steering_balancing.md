2025년 05월 16일

현재 차량은 흰색 실선을 기준으로 중심을 추종하는 방식으로 주행하도록 코드를 작성했습니다.
하지만 도로 주행 중에 검은색 차량이 내려올 경우 흰색 실선이 가려지는 상황이 발생해, 이럴 때는 노란선을 기준으로 fallback 주행하도록 로직을 추가했습니다.

속도를 5~10 정도 올려 테스트해본 결과, 차량이 도로 밖으로 이탈하는 문제가 발생했습니다. 해당 부분은 원인을 분석한 후 추후 수정하여 다시 반영하도록 하겠습니다 .