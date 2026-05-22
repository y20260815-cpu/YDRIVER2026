import subprocess

gcc = r'C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin\arm-none-eabi-g++.exe'
inc = r'c:\Users\thpark\SynologyDrive\6k2jvr_work\prj-stm32ide\vcu\ycbrain\ycb_dmdriver_103rbtx'
src = inc + r'\class\tja1050\tja1050.cpp'

cmd = [
    gcc, src,
    '-mcpu=cortex-m3', '-std=gnu++14', '-DDEBUG', '-DUSE_HAL_DRIVER', '-DSTM32F103xB',
    '-c',
    '-I'+inc+'/Core/Inc',
    '-I'+inc+'/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy',
    '-I'+inc+'/Drivers/STM32F1xx_HAL_Driver/Inc',
    '-I'+inc+'/Drivers/CMSIS/Device/ST/STM32F1xx/Include',
    '-I'+inc+'/Drivers/CMSIS/Include',
    '-Wall',
    '-o', 'NUL'
]

r = subprocess.run(cmd, capture_output=True, text=True)
print('=== STDOUT ===')
print(r.stdout)
print('=== STDERR ===')
print(r.stderr)
print('=== RC:', r.returncode)
