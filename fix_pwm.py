import subprocess

# PWM16.cpp 수정: Apply_PWM_Direct 추가 + PWM_1ms 수정
f = r'c:/Users/thpark/SynologyDrive/6k2jvr_work/prj-stm32ide/vcu/ycbrain/ycb_dmdriver_103rbtx/class/pwm16/PWM16.cpp'
txt = open(f, encoding='utf-8').read()

# 1) Apply_PWM_Direct 함수 삽입 (DisableAllFETs_Dual 닫는 } 직후)
insert_after = '''void PWM16::DisableAllFETs_Dual(){
\thtim2.Instance->CCR1=0;
\thtim2.Instance->CCR2=1024;
\thtim2.Instance->CCR3=0;
\thtim2.Instance->CCR4=1024;
\thtim3.Instance->CCR1=0;
\thtim3.Instance->CCR2=1024;
\thtim3.Instance->CCR3=0;
\thtim3.Instance->CCR4=1024;
}'''

apply_func = '''
// 1ms slew 전용: BEMF 측정/FET 전체 차단 없이 PWM 레지스터만 직접 설정
void PWM16::Apply_PWM_Direct(DoubleF_VALUE fSetPwm)
{
\tint mLeftMotorPwm  = (int)(fminf(fabsf(fSetPwm.f1), 1.0f) * FET_MAX);
\tint mRightMotorPwm = (int)(fminf(fabsf(fSetPwm.f2), 1.0f) * FET_MAX);

\tuint8_t raw_dir1 = (fSetPwm.f1 >= 0.0f) ? 0 : 1;
\tuint8_t raw_dir2 = (fSetPwm.f2 >= 0.0f) ? 0 : 1;

\t// 방향 변경 시에만 FET 차단 + 데드타임
\tif ((raw_dir1 != prev_direction1) || (raw_dir2 != prev_direction2)) {
\t\tDisableAllFETs_Dual();
\t\tdelay_us(5);
\t\tprev_direction1 = raw_dir1;
\t\tprev_direction2 = raw_dir2;
\t}

\tint pwm1,pwm2,pwm3,pwm4,pwm5,pwm6,pwm7,pwm8;
\t// M1
\tif(raw_dir1 == 0){
\t\tpwm1 = mLeftMotorPwm; pwm2 = mLeftMotorPwm + PWM_DIRECT_CONTROL;
\t\tpwm3 = 0; pwm4 = 0;
\t} else {
\t\tpwm3 = mLeftMotorPwm; pwm4 = mLeftMotorPwm + PWM_DIRECT_CONTROL;
\t\tpwm1 = 0; pwm2 = 0;
\t}
\t// M2
\tif(raw_dir2 == 0){
\t\tpwm5 = mRightMotorPwm; pwm6 = mRightMotorPwm + PWM_DIRECT_CONTROL;
\t\tpwm7 = 0; pwm8 = 0;
\t} else {
\t\tpwm7 = mRightMotorPwm; pwm8 = mRightMotorPwm + PWM_DIRECT_CONTROL;
\t\tpwm5 = 0; pwm6 = 0;
\t}

\thtim3.Instance->CCR1 = pwm1;
\thtim3.Instance->CCR2 = pwm2;
\thtim3.Instance->CCR3 = pwm3;
\thtim3.Instance->CCR4 = pwm4;
\thtim2.Instance->CCR1 = pwm5;
\thtim2.Instance->CCR2 = pwm6;
\thtim2.Instance->CCR3 = pwm7;
\thtim2.Instance->CCR4 = pwm8;
}'''

if 'Apply_PWM_Direct' not in txt:
    txt = txt.replace(insert_after, insert_after + apply_func)
    print('Apply_PWM_Direct inserted')
else:
    print('Apply_PWM_Direct already exists')

# 2) PWM_1ms에서 Dual_Motor_set_pwm10 -> Apply_PWM_Direct 교체
old_call = '''\t// 1ms마다 slew 출력을 하드웨어에 반영
\tMOTOR_DIRECTION md_slew = {0, 0};
\tDual_Motor_set_pwm10(md_slew, slew_pwm, pDataClass->disable_pid);'''
new_call = '''\t// 1ms마다 BEMF 없이 PWM 레지스터만 직접 설정
\t// (Dual_Motor_set_pwm10은 내부에서 DisableAllFETs+BEMF 측정을 하므로 1ms 루프에서 사용 불가)
\tApply_PWM_Direct(slew_pwm);'''

if old_call in txt:
    txt = txt.replace(old_call, new_call)
    print('PWM_1ms call replaced')
else:
    print('WARNING: old call not found, manual check needed')

open(f, 'w', encoding='utf-8').write(txt)
print('PWM16.cpp saved')
