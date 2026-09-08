# UART3 실시간 모니터

1. 보드의 UART3(TX: PC10, RX: PC11, GND)를 USB-UART 어댑터에 연결합니다.
2. 펌웨어를 빌드하고 보드에 기록합니다.
3. Chrome 또는 Edge에서 `uart3_monitor.html`을 열고 **시리얼 연결**을 누릅니다.
4. USB-UART 포트를 선택하면 100 ms 주기의 값과 그래프가 표시됩니다.

통신 설정은 **115200 baud, 8 data bits, no parity, 1 stop bit**입니다. UART3에는 아래 고정 형식의 CSV가 전송됩니다.

```text
$VCU,time_ms,source,can_ready,adc1,adc2,fnr1,fnr2,toggle,button,accel_raw,byte3_raw,can_cmd1,can_cmd2,pwm1,pwm2,rpm1,rpm2,fet_temp,motor_temp,voltage,current,error
```

`source`는 GPIO/local=0, CAN=1입니다. `can_cmd1`과 `can_cmd2`는 CAN에서 마지막으로 수신한 좌·우 모터 원본 명령값입니다. `toggle`과 `button`은 CAN/GPIO 입력 비트 필드의 원본 8비트 값입니다. 실제 장비 없이 화면을 확인하려면 **데모 실행**을 누릅니다.
