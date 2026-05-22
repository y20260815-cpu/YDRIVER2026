import subprocess

f = r'c:/Users/thpark/SynologyDrive/6k2jvr_work/prj-stm32ide/vcu/ycbrain/ycb_dmdriver_103rbtx/class/tja1050/tja1050.cpp'
txt = open(f, encoding='utf-8').read()

# 미사용 변수 i 제거
txt = txt.replace('uint8_t i=0,time_out;', 'uint8_t time_out;')

# printf 포맷 수정: 인자 i 추가, %04lX -> %04X (uint8_t는 long이 아님)
old = 'printf("Setup_Data(i) (%04lX)\\r\\n", i, Pub_ID+i);'
new = 'printf("Setup_Data(%d) (%04X)\\r\\n", i, (unsigned int)(Pub_ID+i));'
txt = txt.replace(old, new)

open(f, 'w', encoding='utf-8').write(txt)
print('tja1050.cpp fixed')

# 컴파일 검증
gcc = r'C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin\arm-none-eabi-g++.exe'
inc = r'c:/Users/thpark/SynologyDrive/6k2jvr_work/prj-stm32ide/vcu/ycbrain/ycb_dmdriver_103rbtx'
cmd = [
    gcc, f,
    '-mcpu=cortex-m3', '-std=gnu++14', '-DDEBUG', '-DUSE_HAL_DRIVER', '-DSTM32F103xB',
    '-c',
    '-I'+inc+'/Core/Inc',
    '-I'+inc+'/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy',
    '-I'+inc+'/Drivers/STM32F1xx_HAL_Driver/Inc',
    '-I'+inc+'/Drivers/CMSIS/Device/ST/STM32F1xx/Include',
    '-I'+inc+'/Drivers/CMSIS/Include',
    '-Wall', '-Werror', '-o', 'NUL'
]
r = subprocess.run(cmd, capture_output=True, text=True)
if r.stderr:
    print('STDERR:', r.stderr)
print('RC:', r.returncode)
